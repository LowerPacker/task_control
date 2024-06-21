#ifndef _ROBOT_COM_H_
#define _ROBOT_COM_H_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "task_control_interface/msg/mcu_to_task.hpp"
#include "task_control_interface/msg/task_to_mcu.hpp"
#include "task_control_interface/msg/vision_result.hpp"
#include "task_control_interface/msg/app_cmd.hpp"

namespace task_control {

using namespace task_control_interface::msg;

class RobotCom {
public:
    RobotCom(const std::shared_ptr<rclcpp::Node>& nh);
    RobotCom(const RobotCom&) = delete;
    RobotCom(const RobotCom&&) = delete;
    RobotCom& operator=(const RobotCom&) = delete;
    RobotCom& operator=(const RobotCom&&) = delete;
    virtual ~RobotCom() {}
    virtual void mcu_to_task_callback(const McuToTask::SharedPtr msg) = 0;
    virtual void app_cmd_callback(const AppCmd::SharedPtr msg) = 0;
    virtual void vision_result_callback(const VisionResult::SharedPtr msg) = 0;
    inline rclcpp::Publisher<TaskToMcu>::SharedPtr get_task_to_mcu_publisher() const {
        return task_to_mcu_publisher_;
    }
private:
    rclcpp::Subscription<McuToTask>::SharedPtr mcu_to_task_subscriber_;
    rclcpp::Subscription<AppCmd>::SharedPtr app_cmd_subscriber_;
    rclcpp::Subscription<VisionResult>::SharedPtr vision_result_subscriber_;
    rclcpp::Publisher<TaskToMcu>::SharedPtr task_to_mcu_publisher_;
};

}  // namespace task_control

#endif  // _ROBOT_COM_H_
