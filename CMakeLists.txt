cmake_minimum_required(VERSION 3.5)
project(task_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(task_control_interface REQUIRED)

include_directories(
      include
)

add_executable(task_control 
      src/main.cpp
      src/fsm.cpp
      src/robot_base.cpp
      src/robot_com.cpp
      src/robot_factory.cpp
      src/robot.cpp
)
ament_target_dependencies(task_control 
      rclcpp
      std_msgs
      task_control_interface
)

install(TARGETS
  task_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
