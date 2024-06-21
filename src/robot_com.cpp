#include "robot_com.h"

namespace task_control {

RobotCom::RobotCom(const std::shared_ptr<rclcpp::Node>& nh) {
    task_to_mcu_publisher_ = nh->create_publisher<TaskToMcu>("task_to_mcu", 10);
    mcu_to_task_subscriber_ = nh->create_subscription<McuToTask>("mcu_to_task", 10,
        std::bind(&RobotCom::mcu_to_task_callback, this, std::placeholders::_1));
    app_cmd_subscriber_ = nh->create_subscription<AppCmd>("app_cmd", 10,
        std::bind(&RobotCom::app_cmd_callback, this, std::placeholders::_1));
    vision_result_subscriber_ = nh->create_subscription<VisionResult>("vision_result", 10,
        std::bind(&RobotCom::vision_result_callback, this, std::placeholders::_1));
    motion_action_client_ = rclcpp_action::create_client<Motion>(nh, "motion_action");
}

}  // namespace task_control
      