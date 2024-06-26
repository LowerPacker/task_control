#include <iostream>
#include "robot.h"

namespace task_control {

#define WORK_BRUSH_LEVEL 2
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

        m_isPause = false;
        m_isResume = false;
        m_isStop = false;

        task_to_mcu_msg_.auto_mode = 1;
        m_robot_pos = POS_LEFT;
        m_spin_cnt = 0;
        
        start_work(WORK_BRUSH_LEVEL);
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
        else if(result > MOVE_RESULT_RUNNING) //异常
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
        else if(result > MOVE_RESULT_RUNNING) //异常
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
        else if(result > MOVE_RESULT_RUNNING) //异常
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
        if(result > MOVE_RESULT_RUNNING) //异常
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
        else if(result > MOVE_RESULT_RUNNING) //异常
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
        else if(result > MOVE_RESULT_RUNNING) //异常
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
        else if(result > MOVE_RESULT_RUNNING) //异常
        {
            RCLCPP_INFO(nh_->get_logger(), "line: %d goto_state FSM_STATE_FINISH", __LINE__);
            robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_FINISH);
        }

        return true;
    });

    robot_fsm_.init();
}

bool Robot::do_normal() {

    //更新主控参数
    deal_update_task_msg();
    
    //处理急停
    deal_emer_event();
    
    //处理异常，并根据异常控制灯和蜂鸣器
    deal_fault_event();

    //处理自动作业时的命令
    deal_auto_work_cmd();

    return spin_once();
}

bool Robot::start_work(int brush_v_level) {
    task_to_mcu_msg_.suction_cup = 1;      // 放吸盘
    task_to_mcu_msg_.brush_v_level = brush_v_level;   
    // get_task_to_mcu_publisher()->publish(task_to_mcu_msg_);   
	return true;
}

bool Robot::stop_work() {
    task_to_mcu_msg_.suction_cup = 0;        // 不放吸盘
    task_to_mcu_msg_.brush_v_level = 0;      // 辊刷速度置为0
    // get_task_to_mcu_publisher()->publish(task_to_mcu_msg_);
	return true;
}

bool Robot::stop_move() {
    // 停止机器运动
    // todu
    cancel_goal();
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
    // std::cout << "-------mcu_to_task_callback!-----------" << std::endl;
    m_mcu_to_task_msg_ = msg;
}

void Robot::motion_to_mcu_callback(const MotionToMcu::SharedPtr msg)
{
    // std::cout << "-------motion_to_mcu_callback!-----------" << std::endl;
    m_motion_to_mcu_msg_ = msg;
}

void Robot::vision_result_callback(const VisionResult::SharedPtr msg) 
{
    // std::cout << "-------vision_result_callback!-----------" << std::endl;
    m_vision_result_msg_ = msg;
}

// task_mode: 0--spin 1--advance 2--recoil
void Robot::set_robot_move(int task_mode, float aim_x, float aim_y, float aim_yaw)
{
    send_goal(task_mode, aim_x, aim_y, aim_yaw);
}

void Robot::deal_update_task_msg()
{
    bool bFlag = false;

    //有更新
    if(task_to_mcu_msg_.emergency_stop != task_to_mcu_msg_buf_.emergency_stop || 
    task_to_mcu_msg_.suction_cup != task_to_mcu_msg_buf_.suction_cup || 
    task_to_mcu_msg_.led_control != task_to_mcu_msg_buf_.led_control || 
    task_to_mcu_msg_.buzzer_control != task_to_mcu_msg_buf_.buzzer_control || 
    task_to_mcu_msg_.brush_v_level != task_to_mcu_msg_buf_.brush_v_level || 
    task_to_mcu_msg_.auto_mode != task_to_mcu_msg_buf_.auto_mode
    )
    {
        bFlag = true;
        task_to_mcu_msg_buf_ = task_to_mcu_msg_;
    }

    if(bFlag)
    {
        get_task_to_mcu_publisher()->publish(task_to_mcu_msg_);
    }
}

void Robot::deal_emer_event()
{
    if(get_emer_stop_status())
    {
        if(++m_emer_cnt >= 5)
        {
            m_emer_flag = true;
        }
    }
    else
    {
        m_emer_flag = false;
    }

    //急停触发
    if(m_emer_flag)
    {
        emr();
    }
    else
    {
        emr_restore();
    }
}

void Robot::deal_fault_event()
{
    if(get_exception_code() || m_emer_flag || get_motor_status() || get_brush_status())
    {
        m_fault_level = FAULT_LEVEL_ERROR_1;
        task_to_mcu_msg_.led_control = 5;
        task_to_mcu_msg_.buzzer_control = 1;
        stop_work();
        stop_move();
    }
    else if (get_bat_vol() <= 5) //电量故障
    {
        m_fault_level = FAULT_LEVEL_ERROR_2;
        task_to_mcu_msg_.led_control = 5;
        task_to_mcu_msg_.buzzer_control = 1;
    }
    // else if() //警告
    // {

    // }
    else if(get_bat_vol() <= 20) //电量警告
    {
        m_fault_level = FAULT_LEVEL_WARNN_2;
        task_to_mcu_msg_.led_control = 3;
        task_to_mcu_msg_.buzzer_control = 3;
    }
    else if(task_to_mcu_msg_.auto_mode == 1) //自动模式
    {
        m_fault_level = FAULT_LEVEL_NORMA_2;
        task_to_mcu_msg_.led_control = 1;
        task_to_mcu_msg_.buzzer_control = 0;
    }
    else //手动模式
    {
        m_fault_level = FAULT_LEVEL_NORMA_1;
        task_to_mcu_msg_.led_control = 2;
        task_to_mcu_msg_.buzzer_control = 0;
    }
    
}

void Robot::deal_auto_work_cmd()
{
    //自动作业运行中
    if(is_fsm_running())
    {
        if(m_isPause && !m_isResume)
        {
            robot_fsm_.pause();
            stop_work();
            stop_move();
        }

        if(!m_isPause && m_isResume) 
        {
            //机器在运动中暂停，则需重新执行上一个状态
            if(get_move_result() == MOVE_RESULT_RUNNING)
            {
                robot_fsm_.init(robot_fsm_.get_last_state());
            }
            else
            {
                robot_fsm_.restart();
            }
            start_work(WORK_BRUSH_LEVEL);
        }

        if(m_isStop)
        {
            robot_fsm_.init();
            stop_work();
            stop_move();
        }
    }
}

bool Robot::is_fsm_running()
{
    return (robot_fsm_.get_state() != FSM_STATE_IDEL);
}


}  // namespace task_control
