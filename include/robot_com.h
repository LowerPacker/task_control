#ifndef _ROBOT_COM_H_
#define _ROBOT_COM_H_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "task_control_interface/msg/task_control.hpp"
#include "task_control_interface/msg/mcu_info.hpp"
#include "task_control_interface/msg/app_cmd.hpp"

namespace task_control {

class RobotCom {
public:
    RobotCom(const std::shared_ptr<rclcpp::Node>& nh);
    RobotCom(const RobotCom&) = delete;
    RobotCom(const RobotCom&&) = delete;
    RobotCom& operator=(const RobotCom&) = delete;
    RobotCom& operator=(const RobotCom&&) = delete;
    virtual ~RobotCom() {}
    virtual void mcu_info_callback(const task_control_interface::msg::McuInfo::SharedPtr msg) = 0;
    virtual void app_cmd_callback(const task_control_interface::msg::AppCmd::SharedPtr msg) = 0;
    inline rclcpp::Publisher<task_control_interface::msg::TaskControl>::SharedPtr get_task_control_publisher() const {
        return task_control_publisher_;
    }
private:
    rclcpp::Subscription<task_control_interface::msg::McuInfo>::SharedPtr mcu_info_subscriber_;
    rclcpp::Subscription<task_control_interface::msg::AppCmd>::SharedPtr app_cmd_subscriber_;
    rclcpp::Publisher<task_control_interface::msg::TaskControl>::SharedPtr task_control_publisher_;
};

}  // namespace task_control

#endif  // _ROBOT_COM_H_
