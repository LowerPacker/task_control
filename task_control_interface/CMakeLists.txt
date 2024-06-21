cmake_minimum_required(VERSION 3.8)
project(task_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# message(STATUS "当前目录是: ${CMAKE_CURRENT_SOURCE_DIR}")
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/McuFeedback.msg
  msg/McuInfo.msg
  msg/MotionExcution.msg
  msg/TaskControl.msg
  msg/AppCmd.msg
  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
  DEPENDENCIES geometry_msgs
)

ament_package()