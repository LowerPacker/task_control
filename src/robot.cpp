#include <iostream>
#include "robot.h"

namespace task_control {

#define PI 3.1415926535
#define D_ADVANCE_VALUE 0.5
#define D_RECOIL_VALUE 0.5

Robot::Robot(const std::shared_ptr<rclcpp::Node>& nh) : RobotBase(nh), RobotCom(nh) {
    init(nh);

    m_mcu_to_motion_msg_ = std::make_shared<McuToMotion>();
    m_mcu_to_task_msg_ = std::make_shared<McuToTask>();
    m_motion_to_mcu_msg_ = std::make_shared<MotionToMcu>();
    m_vision_result_msg_ = std::make_shared<VisionResult>();
}

void Robot::init(const std::shared_ptr<rclcpp::Node>& nh) {
    // 空闲状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_IDEL, [this](FSM* f) {
        if (task_ == E_ROBOT_TASK::TASK_AUTO) {
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_READY);
        }
        return true;
    });
    // 准备状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_READY, [this](FSM* f) {

        m_robot_pos = POS_LEFT;
        m_spin_cnt = 0;
        
        start_work(2);
        get_task_to_mcu_publisher()->publish(task_to_mcu_msg_);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_CHECK_MOTOR);
        return true;
    });
    // 作业状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_WORK, [this](FSM* f) {
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_PAUSE);
        return true;
    });
    // 暂停状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_PAUSE, [this](FSM* f) {
        stop_work();
        stop_move();
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_CONTINUE);
        return true;
    });
    // 继续状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_CONTINUE, [this](FSM* f) {
        if (0) {
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_READY);
        }

        return true;
    });
    // 停止状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_STOP, [this](FSM* f) {
        stop_work();
        stop_move();
        get_task_to_mcu_publisher()->publish(task_to_mcu_msg_);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        return true;
    });
    // 结束状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_FINISH, [this](FSM* f) {
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_IDEL);
        return true;
    });
    
    // 检测电机运行状态
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_CHECK_MOTOR, [this](FSM* f) {
        
        //电机异常
        if(get_motor_status() || get_brush_status() || get_drop_sign())
        {
            //机器异常
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_STOP);
        }
        else
        {
            //机器正常，开始工作
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FIRST_MOVE", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FIRST_MOVE);
        }
        return true;
    });
    
    // 第一次往前走
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_FIRST_MOVE, [this](FSM* f) {
        
        //下发一直往前走
        set_robot_move(MOVE_ADVANCE);
        RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FIRST_MOVE_WAIT_RESULT", __LINE__);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FIRST_MOVE_WAIT_RESULT);
        return true;
    });
    
    // 第一次往前走等待结果
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_FIRST_MOVE_WAIT_RESULT, [this](FSM* f) {
        
        //监测跌落标志，触发则取消运控任务并跳到下一个状态
        if(get_drop_sign())
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FIRST_RECOIL", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FIRST_RECOIL);
        }

        auto result = get_move_result();
        if(result == MOVE_RESULT_RUNNING) //执行中
        {

        }
        else //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });
    
    // 第一次后退
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_FIRST_RECOIL, [this](FSM* f) {
        
        //
        set_robot_move(MOVE_RECOIL, D_RECOIL_VALUE);
        RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FIRST_RECOIL_WAIT_RESULT", __LINE__);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FIRST_RECOIL_WAIT_RESULT);
        return true;
    });
    
    // 第一次后退等待结果
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_FIRST_RECOIL_WAIT_RESULT, [this](FSM* f) {
        
        auto result = get_move_result();
        if(result == MOVE_RESULT_SUCCESS) //成功
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FIRST_SPIN", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FIRST_SPIN);
        }
        else if(result == MOVE_RESULT_RUNNING) //执行中
        {

        }
        else //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });
    
    // 第一次旋转
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_FIRST_SPIN, [this](FSM* f) {
        
        //
        set_robot_move(MOVE_SPIN, 0, 0, (-1)*PI/2);
        RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FIRST_RECOIL_WAIT_RESULT", __LINE__);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FIRST_RECOIL_WAIT_RESULT);
        return true;
    });
    
    // 第一次旋转等待结果
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_FIRST_RECOIL_WAIT_RESULT, [this](FSM* f) {
        
        auto result = get_move_result();
        if(result == MOVE_RESULT_SUCCESS) //成功
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_ADVANCE", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_ADVANCE);
        }
        else if(result == MOVE_RESULT_RUNNING) //执行中
        {

        }
        else //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });
    
    // 一直前进
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_ADVANCE, [this](FSM* f) {
        
        //
        set_robot_move(MOVE_ADVANCE);
        RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_ADVANCE_WAIT_RESULT", __LINE__);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_ADVANCE_WAIT_RESULT);
        return true;
    });
    
    // 一直前进等待结果
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_ADVANCE_WAIT_RESULT, [this](FSM* f) {
        
        //监测跌落标志，触发则取消运控任务并跳到下一个状态
        if(get_drop_sign())
        {
            //更新机器在光伏板上的位置
            m_robot_pos = (m_robot_pos == POS_LEFT ? POS_RIGHT : POS_LEFT);
            
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_RECOIL_FIXED", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_RECOIL_FIXED);
        }

        auto result = get_move_result();
        if(result == MOVE_RESULT_RUNNING) //执行中
        {

        }
        else //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });
    
    // 前进固定距离
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_ADVANCE_FIXED, [this](FSM* f) {
        
        //
        set_robot_move(MOVE_RECOIL, D_RECOIL_VALUE);
        RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_ADVANCE_FIXED_WAIT_RESULT", __LINE__);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_ADVANCE_FIXED_WAIT_RESULT);
        return true;
    });
    
    // 前进固定距离等待结果
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_ADVANCE_FIXED_WAIT_RESULT, [this](FSM* f) {
        
        //跌落标志触发
        if(get_drop_sign())
        {
            //取消运控任务


            // if() //行走距离小于某个值，任务完成
            // {
            //     robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
            // }
            // else //完成最后一行
            // {
            //     robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_RECOIL_FIXED);
            // }
        }

        auto result = get_move_result();
        if(result == MOVE_RESULT_SUCCESS) //成功
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_SPIN", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_SPIN);
        }
        else if(result == MOVE_RESULT_RUNNING) //执行中
        {

        }
        else //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });
    
    // 后退固定距离
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_RECOIL_FIXED, [this](FSM* f) {
        
        //
        set_robot_move(MOVE_RECOIL, D_RECOIL_VALUE);
        RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_RECOIL_FIXED_WAIT_RESULT", __LINE__);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_RECOIL_FIXED_WAIT_RESULT);
        return true;
    });
    
    // 后退固定距离等待结果
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_RECOIL_FIXED_WAIT_RESULT, [this](FSM* f) {
        
        auto result = get_move_result();
        if(result == MOVE_RESULT_SUCCESS) //成功
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_SPIN", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_SPIN);
        }
        else if(result == MOVE_RESULT_RUNNING) //执行中
        {

        }
        else //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });
    
    // 旋转
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_SPIN, [this](FSM* f) {
        
        float spin_angle = 0;
        if(m_robot_pos == POS_LEFT) //机器在左边，逆时针旋转
        {
            spin_angle = PI / 2;
        }
        else //机器在右边，顺时针旋转
        {
            spin_angle = (-1) * PI / 2;
        }

        if(spin_angle != 0)
        {
            set_robot_move(MOVE_SPIN, 0, 0, spin_angle);
        }

        RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_SPIN_WAIT_RESULT", __LINE__);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_SPIN_WAIT_RESULT);
        return true;
    });
    
    // 旋转等待结果
    robot_fsm_.add_state(E_FSM_STATE::FSM_STATE_SPIN_WAIT_RESULT, [this](FSM* f) {
        
        auto result = get_move_result();
        if(result == MOVE_RESULT_SUCCESS) //成功
        {
            if(++m_spin_cnt == 1) //第一次旋转，换行，走固定距离
            {
                RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_ADVANCE_FIXED", __LINE__);
                robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_ADVANCE_FIXED);
            }
            else if(++m_spin_cnt == 2) //第二次旋转，开始一直往前走
            {
                RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_ADVANCE", __LINE__);
                robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_ADVANCE);
                m_spin_cnt = 0;
            }
        }
        else if(result == MOVE_RESULT_RUNNING) //执行中
        {

        }
        else //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });

    robot_fsm_.init();
}

bool Robot::do_normal() {
    return spin_once();
}

bool Robot::start_work(int brush_v_level) {
    task_to_mcu_msg_.suction_cup = 1;      // 放吸盘
    task_to_mcu_msg_.brush_v_level = brush_v_level;      
	return true;
}

bool Robot::stop_work() {
    task_to_mcu_msg_.suction_cup = 0;        // 不放吸盘
    task_to_mcu_msg_.brush_v_level = 0;      // 辊刷速度置为0
	return true;
}

bool Robot::stop_move() {
    // 停止机器运动
    // todu
    return true;
}

bool Robot::emr() {
    task_to_mcu_msg_.emergency_stop = 1;   // 急停
    stop_work();
    stop_move();
    return true;
}

bool Robot::emr_restore() {
    task_to_mcu_msg_.emergency_stop = 0;   // 不急停
    return true;
}

void Robot::app_cmd_callback(const AppCmd::SharedPtr msg) {
    // 手自动切换
   if (robot_mode_ != msg->mode_cmd) {
        robot_mode_ = msg->mode_cmd;
        std::cout << "robot mode:" << robot_mode_ << std::endl;
        stop_work();
   }
   // 急停
   if (msg->emr_cmd == 1) { 
        if (robot_state_ != E_ROBOT_STATE::STATE_EMR) {
            robot_state_ = E_ROBOT_STATE::STATE_EMR;
            emr();
        }
   }
   // 急停恢复
   if (msg->emr_cmd == 2) {
        if (robot_state_ == E_ROBOT_STATE::STATE_EMR) {
            robot_state_ = STATE_EMR_RESTORE;
            emr_restore();
        }
   }
    get_task_to_mcu_publisher()->publish(task_to_mcu_msg_);
}

void Robot::mcu_to_motion_callback(const McuToMotion::SharedPtr msg)
{
    // std::cout << "-------mcu_to_motion_callback!-----------" << std::endl;
    m_mcu_to_motion_msg_ = msg;
}

void Robot::mcu_to_task_callback(const McuToTask::SharedPtr msg) 
{
    m_mcu_to_task_msg_ = msg;
}

void Robot::motion_to_mcu_callback(const MotionToMcu::SharedPtr msg)
{
    // std::cout << "-------motion_to_mcu_callback!-----------" << std::endl;
    m_motion_to_mcu_msg_ = msg;
}

void Robot::vision_result_callback(const VisionResult::SharedPtr msg) 
{
    
}

// task_mode: 0--spin 1--advance 2--recoil
void Robot::set_robot_move(int task_mode, float aim_x, float aim_y, float aim_yaw)
{
    send_goal(task_mode, aim_x, aim_y, aim_yaw);
}


}  // namespace task_control
