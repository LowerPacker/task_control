#include "robot_factory.h"

using namespace task_control;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("task_control");
	std::shared_ptr<Robot> robot_ptr;
    if (argc > 1) {
        robot_ptr = std::shared_ptr<Robot>(RobotFactory::get_robot_instance(node, argv[1]));
    } else {
        robot_ptr = std::shared_ptr<Robot>(RobotFactory::get_robot_instance(node, "robot"));
    }
    if (robot_ptr == nullptr) {
        return 0;
    }

	rclcpp::Rate rate(50);
	while (rclcpp::ok()) {
		robot_ptr->do_normal();
		rclcpp::spin_some(node);
		rate.sleep();
	}
	
    rclcpp::shutdown();
	return 0;
}

