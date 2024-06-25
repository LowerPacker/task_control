#ifndef _ROBOT_COM_H_
#define _ROBOT_COM_H_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_control_interface/msg/mcu_to_task.hpp"
#include "task_control_interface/msg/task_to_mcu.hpp"
#include "task_control_interface/msg/vision_result.hpp"
#include "task_control_interface/msg/app_cmd.hpp"
#include "task_control_interface/msg/mcu_to_motion.hpp"
#include "task_control_interface/msg/motion_to_mcu.hpp"
#include "task_control_interface/action/motion.hpp"

namespace task_control {

using namespace task_control_interface::msg;
using namespace task_control_interface::action;
using GoalHandleMotion = rclcpp_action::ClientGoalHandle<Motion>;

class RobotCom {
public:
    RobotCom(const std::shared_ptr<rclcpp::Node>& nh);
    RobotCom(const RobotCom&) = delete;
    RobotCom(const RobotCom&&) = delete;
    RobotCom& operator=(const RobotCom&) = delete;
    RobotCom& operator=(const RobotCom&&) = delete;
    virtual ~RobotCom() {}

    virtual void mcu_to_motion_callback(const McuToMotion::SharedPtr msg) = 0;
    virtual void mcu_to_task_callback(const McuToTask::SharedPtr msg) = 0;
    virtual void motion_to_mcu_callback(const MotionToMcu::SharedPtr msg) = 0;
    virtual void app_cmd_callback(const AppCmd::SharedPtr msg) = 0;
    virtual void vision_result_callback(const VisionResult::SharedPtr msg) = 0;

    void goal_response_callback(std::shared_future<GoalHandleMotion::SharedPtr> future);
    void feedback_callback(GoalHandleMotion::SharedPtr, const std::shared_ptr<const Motion::Feedback> feedback);
    void result_callback(const GoalHandleMotion::WrappedResult& result);
    void send_goal();

    inline rclcpp::Publisher<TaskToMcu>::SharedPtr get_task_to_mcu_publisher() const {
        return task_to_mcu_publisher_;
    }

private:
    rclcpp::Subscription<McuToTask>::SharedPtr mcu_to_task_subscriber_;
    rclcpp::Subscription<AppCmd>::SharedPtr app_cmd_subscriber_;
    rclcpp::Subscription<VisionResult>::SharedPtr vision_result_subscriber_;
    rclcpp::Subscription<McuToMotion>::SharedPtr mcu_to_motion_subscriber_;
    rclcpp::Subscription<MotionToMcu>::SharedPtr motion_to_mcu_subscriber_;
    rclcpp::Publisher<TaskToMcu>::SharedPtr task_to_mcu_publisher_;
    rclcpp_action::Client<Motion>::SharedPtr motion_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Node> nh_;
};

}  // namespace task_control

#endif  // _ROBOT_COM_H_
