#include "robot_factory.h"
#include "robot.h"

namespace task_control {

Robot* RobotFactory::get_robot_instance(const std::shared_ptr<rclcpp::Node>& nh, const std::string& name) {
    typedef std::function<Robot* (const std::shared_ptr<rclcpp::Node>& nh)> create_robot_func;
    std::string robot_name(name);
    static std::map<std::string, create_robot_func> robots = {
        {"robot", [](const std::shared_ptr<rclcpp::Node>& nh){return new Robot(nh);}},
    };

    transform(robot_name.begin(), robot_name.end(), robot_name.begin(), ::tolower);
    if(robots.find(robot_name) == robots.end()) {
        return NULL;
    }
    return robots[robot_name](nh);
}

}  // namespace task_control

