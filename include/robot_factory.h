#ifndef _ROBOT_FACTORY_H_
#define _ROBOT_FACTORY_H_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot.h"

namespace task_control {

class RobotFactory {
public:
    static Robot* get_robot_instance(const std::shared_ptr<rclcpp::Node>& nh, const std::string& robot_name);
};

}  // namespace task_control

#endif // _ROBOT_FACTORY_H_
