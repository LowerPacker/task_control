#include <iostream>
#include "robot.h"

namespace task_control {

Robot::Robot(const std::shared_ptr<rclcpp::Node>& nh) : RobotBase(nh), RobotCom(nh) {
    init(nh);
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
        start_work(2);
        get_task_to_mcu_publisher()->publish(task_to_mcu_msg_);
        robot_fsm_.goto_state(E_FSM_STATE::FSM_STATE_WORK);
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

void Robot::mcu_to_task_callback(const McuToTask::SharedPtr msg) {

}


}  // namespace task_control
