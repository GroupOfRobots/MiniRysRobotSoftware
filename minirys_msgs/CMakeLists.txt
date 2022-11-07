cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(minirys_msgs
  VERSION 0.1.1
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AngularPose.msg
  msg/BatteryStatus.msg
  msg/MotorCommand.msg
  msg/MotorDriverStatus.msg
  msg/RTTestResult.msg
  srv/SetPose.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(
  rosidl_default_generators
  std_msgs
  geometry_msgs
)

ament_package()
