cmake_minimum_required(VERSION 3.8)
project(followers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs)
find_package(ament_cmake_python REQUIRED)
find_package(pid_regulator REQUIRED)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include
  ${ament_INCLUDE_DIRS}
)

add_library(CombinedFollowerNode
  src/nodes/CombinedFollowerNode.cpp
)
ament_target_dependencies(CombinedFollowerNode
  rclcpp std_msgs sensor_msgs geometry_msgs
)

add_library(WallFollowerNode
  src/nodes/WallFollowerNode.cpp
)
ament_target_dependencies(WallFollowerNode
  rclcpp std_msgs sensor_msgs geometry_msgs pid_regulator
)

add_executable(wallfollower src/executables/wallfollower.cpp)
target_link_libraries(wallfollower WallFollowerNode)

add_executable(combinedFollower src/executables/combinedFollower.cpp)
target_link_libraries(combinedFollower CombinedFollowerNode)

install(TARGETS
  wallfollower
  DESTINATION lib/${PROJECT_NAME}
)

 install(TARGETS
  combinedFollower
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(followers)

install(PROGRAMS
  followers/linefollower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  followers/pid.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  followers/linefollower_no_cam.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
