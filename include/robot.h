#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <memory>
#include <unordered_map>  
#include <functional> 
#include "rclcpp/rclcpp.hpp"
#include "robot_base.h"
#include "robot_com.h"

namespace task_control {

enum E_ROBOT_MODE
{
    MANUAL = 1,
    AUTO
};

enum E_ROBOT_TASK
{
    TASK_NULL = 0,    // 无任务
    TASK_MANUAL,      // 手动任务
    TASK_AUTO,        // 自动任务
};

enum E_ROBOT_STATE
{
    STATE_IDEL = 0,             // 空闲
    STATE_PAUSE,                // 暂停
    STATE_STOP,                 // 停止
    STATE_AUTO_WORKING,         // 自动作业
    STATE_MANUAL_WORKING,       // 手动作业
    STATE_FAULT,                // 故障
    STATE_EMR,                  // 急停
    STATE_EMR_RESTORE,          // 急停复位
};


enum E_FSM_STATE
{
    FSM_STATE_IDEL = 0,	        // 空闲
    FSM_STATE_READY,       	    // 准备
    FSM_STATE_WORK,        	    // 作业
    FSM_STATE_PAUSE,		    // 暂停
    FSM_STATE_CONTINUE,		    // 继续
    FSM_STATE_STOP,             // 停止
    FSM_STATE_FINISH,	        // 结束
};

class Robot final : public RobotBase, public RobotCom {
public:
    Robot(const std::shared_ptr<rclcpp::Node>& nh);
    Robot(const Robot&) = delete;
    Robot(const Robot&&) = delete;
    Robot& operator=(const Robot&) = delete;
    Robot& operator=(const Robot&&) = delete;
    ~Robot() {}

    void init(const std::shared_ptr<rclcpp::Node>& nh) override;
    bool do_normal() override;                                                                    			
    bool start_work() override;
    bool stop_work() override;
    bool stop_move() override;
    bool emr() override;
    bool emr_restore() override;
    void mcu_info_callback(const task_control_interface::msg::McuInfo::SharedPtr msg) override;
    void app_cmd_callback(const task_control_interface::msg::AppCmd::SharedPtr msg) override;

private:
    task_control_interface::msg::TaskControl task_control_msg_;
	int robot_mode_ = E_ROBOT_MODE::MANUAL;
    int robot_state_ = E_ROBOT_STATE::STATE_IDEL;
    int task_ = E_ROBOT_TASK::TASK_NULL;
};

}  // namespace task_control

#endif // _ROBOT_H_
