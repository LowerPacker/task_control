#include "robot_com.h"

namespace task_control {

RobotCom::RobotCom(const std::shared_ptr<rclcpp::Node>& nh) {
    task_control_publisher_ = nh->create_publisher<task_control_interface::msg::TaskControl>("task_control", 10);
    mcu_info_subscriber_ = nh->create_subscription<task_control_interface::msg::McuInfo>("mcu_info", 
        10,
        std::bind(&RobotCom::mcu_info_callback, this, std::placeholders::_1));
    app_cmd_subscriber_ = nh->create_subscription<task_control_interface::msg::AppCmd>("app_cmd", 
        10,
        std::bind(&RobotCom::app_cmd_callback, this, std::placeholders::_1));
}

}  // namespace task_control
      