#include <chrono>
#include "robot_com.h"

namespace task_control {

RobotCom::RobotCom(const std::shared_ptr<rclcpp::Node>& nh) : nh_(nh) {
    task_to_mcu_publisher_ = nh->create_publisher<TaskToMcu>("task_to_mcu", 10);

    mcu_to_task_subscriber_ = nh->create_subscription<McuToTask>("mcu_to_task", 10,
        std::bind(&RobotCom::mcu_to_task_callback, this, std::placeholders::_1));

    app_cmd_subscriber_ = nh->create_subscription<AppCmd>("app_cmd", 10,
        std::bind(&RobotCom::app_cmd_callback, this, std::placeholders::_1));

    vision_result_subscriber_ = nh->create_subscription<VisionResult>("vision_result", 10,
        std::bind(&RobotCom::vision_result_callback, this, std::placeholders::_1));

    mcu_to_motion_subscriber_ = nh->create_subscription<McuToMotion>("mcu_to_motion", 10,
        std::bind(&RobotCom::mcu_to_motion_callback, this, std::placeholders::_1));

    motion_to_mcu_subscriber_ = nh->create_subscription<MotionToMcu>("motion_to_mcu", 10,
        std::bind(&RobotCom::motion_to_mcu_callback, this, std::placeholders::_1));

    motion_action_client_ = rclcpp_action::create_client<Motion>(nh, "motion_action");
}

void RobotCom::send_goal(int task_mode, float aim_x, float aim_y, float aim_yaw) {
    // using namespace std::placeholders;

    // this->timer_->cancel();

    // this->goal_done_ = false;

    // if (!this->motion_action_client_) {
    //     RCLCPP_ERROR(nh_->get_logger(), "Action client not initialized");
    // }

    // if (!this->motion_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    //     RCLCPP_ERROR(nh_->get_logger(), "Action server not available after waiting");
    //     this->goal_done_ = true;
    //     return;
    // }

    // auto goal_msg = Motion::Goal();
    // goal_msg.order = 10;


    auto goal_msg = Motion::Goal();
    goal_msg.start_pose_x = 0;
    goal_msg.start_pose_y = 0;
    goal_msg.start_atti_yaw = 0;
    goal_msg.aim_pose_x = aim_x;
    goal_msg.aim_pose_y = aim_y;
    goal_msg.aim_atti_yaw = aim_yaw;
    goal_msg.linear_vel_max = 0.5;
    goal_msg.angular_rate_max = 0.3;
    goal_msg.task_mode = task_mode;

    //移动固定距离
    if(aim_x != 0 || aim_y != 0 || aim_yaw != 0)
    {
        goal_msg.need_stop = 1;
    }
    else
    {
        goal_msg.need_stop = 0;
    }

    RCLCPP_INFO(nh_->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Motion>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&RobotCom::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&RobotCom::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&RobotCom::result_callback, this, std::placeholders::_1);
    auto goal_handle_future = this->motion_action_client_->async_send_goal(goal_msg, send_goal_options);

    m_move_result = MOVE_RESULT_RUNNING;
}

void RobotCom::goal_response_callback(std::shared_future<GoalHandleMotion::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(nh_->get_logger(), "Goal was rejected by server");
      m_move_result = MOVE_RESULT_REJECT;
    } else {
      RCLCPP_INFO(nh_->get_logger(), "Goal accepted by server, waiting for result");
      m_move_result = MOVE_RESULT_RUNNING;
    }
}

void RobotCom::feedback_callback(GoalHandleMotion::SharedPtr, const std::shared_ptr<const Motion::Feedback> feedback)  {
    // RCLCPP_INFO(nh_->get_logger(), "Next number in sequence received: %", feedback->sequence.back());
}

void RobotCom::result_callback(const GoalHandleMotion::WrappedResult& result) {
    // this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        m_move_result = MOVE_RESULT_SUCCESS;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(nh_->get_logger(), "Goal was aborted");
        m_move_result = MOVE_RESULT_ABORTED;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(nh_->get_logger(), "Goal was canceled");
        m_move_result = MOVE_RESULT_FAIL;
        return;
      default:
        m_move_result = MOVE_RESULT_ERROR;
        RCLCPP_ERROR(nh_->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(nh_->get_logger(), "Result received");
    // for (auto number : result.result->sequence) {
    //   RCLCPP_INFO(nh_->get_logger(), "%", number);
    // }
}

//1--success 2--running 3--fail
int RobotCom::get_move_result()
{
    return m_move_result;
}

}  // namespace task_control
      