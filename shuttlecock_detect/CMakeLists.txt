cmake_minimum_required(VERSION 3.10)
project(shuttlecock_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(ncnn_DIR ../ncnn/lib/cmake/ncnn )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs)
find_package(ament_cmake_python REQUIRED)
find_package(ncnn REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_ros2
    std_msgs
    std_srvs
    rclcpp
    rclcpp_action
    sensor_msgs
    geometry_msgs
    nav_msgs
    nav2_msgs
    tf2_ros
    btcpp_ros2_interfaces
    )

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


include_directories(include ${ament_INCLUDE_DIRS} ../minirys_ros2/include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(PIDRegulator
  ../minirys_ros2/src/helpers/PIDRegulator.cpp
)
add_library(Yolov7
  src/helpers/Yolov7.cpp
)
target_link_libraries(Yolov7 ncnn)

ament_target_dependencies(Yolov7
OpenCV
)

add_library(DetectorNode
  src/nodes/DetectorNode.cpp 
)
ament_target_dependencies(DetectorNode
rclcpp std_msgs sensor_msgs cv_bridge OpenCV ncnn nav_msgs nav2_msgs rclcpp_action geometry_msgs tf2_ros
)
target_link_libraries(DetectorNode Yolov7 PIDRegulator)

add_library(DistancesNode
  src/nodes/DistancesNode.cpp 
)
ament_target_dependencies(DistancesNode
rclcpp std_msgs sensor_msgs cv_bridge OpenCV ncnn nav_msgs nav2_msgs rclcpp_action geometry_msgs tf2_ros
)
target_link_libraries(DistancesNode Yolov7)

add_library(DeliverShuttlecockServer
src/servers/DeliverShuttlecockServer.cpp
)
ament_target_dependencies(DeliverShuttlecockServer ${THIS_PACKAGE_DEPS})

add_library(BackUpServer
src/servers/BackUpServer.cpp
)
ament_target_dependencies(BackUpServer ${THIS_PACKAGE_DEPS})

add_library(GetToShuttlecockServer
src/servers/GetToShuttlecockServer.cpp
)
ament_target_dependencies(GetToShuttlecockServer ${THIS_PACKAGE_DEPS})

add_library(PickShuttlecockServer
src/servers/PickShuttlecockServer.cpp
)
ament_target_dependencies(PickShuttlecockServer ${THIS_PACKAGE_DEPS})

add_library(RotateServer
src/servers/RotateServer.cpp
)
ament_target_dependencies(RotateServer ${THIS_PACKAGE_DEPS})
target_link_libraries(RotateServer PIDRegulator)

add_library(SearchingShuttlecockServer
src/servers/SearchingShuttlecockServer.cpp
)
ament_target_dependencies(SearchingShuttlecockServer ${THIS_PACKAGE_DEPS})

add_executable(server_executor
  src/executables/server_executor.cpp
)
target_link_libraries(server_executor
  SearchingShuttlecockServer
  RotateServer
  PickShuttlecockServer
  GetToShuttlecockServer
  BackUpServer
  DeliverShuttlecockServer
  DistancesNode
)
ament_target_dependencies(server_executor ${THIS_PACKAGE_DEPS})

add_library(BackUpAction
src/actions/BackUpAction.cpp
)
ament_target_dependencies(BackUpAction ${THIS_PACKAGE_DEPS})

add_library(DeliverShuttlecockAction
src/actions/DeliverShuttlecockAction.cpp
)
ament_target_dependencies(DeliverShuttlecockAction ${THIS_PACKAGE_DEPS})

add_library(GetToShuttlecockAction
src/actions/GetToShuttlecockAction.cpp
)
ament_target_dependencies(GetToShuttlecockAction ${THIS_PACKAGE_DEPS})

add_library(PickShuttlecockAction
src/actions/PickShuttlecockAction.cpp
)
ament_target_dependencies(PickShuttlecockAction ${THIS_PACKAGE_DEPS})

add_library(RotateAction
src/actions/RotateAction.cpp
)
ament_target_dependencies(RotateAction ${THIS_PACKAGE_DEPS})

add_library(SearchingShuttlecockAction
src/actions/SearchingShuttlecockAction.cpp
)
ament_target_dependencies(SearchingShuttlecockAction ${THIS_PACKAGE_DEPS})

add_executable(client_executor
  src/executables/client_executor.cpp
)
target_link_libraries(client_executor
  SearchingShuttlecockAction
  RotateAction
  PickShuttlecockAction
  GetToShuttlecockAction
  BackUpAction
  DeliverShuttlecockAction
)
ament_target_dependencies(client_executor ${THIS_PACKAGE_DEPS})


add_executable(yolov7_time_test src/executables/yolov7_time_test.cpp)
target_link_libraries(yolov7_time_test Yolov7 ncnn)
ament_target_dependencies(yolov7_time_test OpenCV ncnn)

add_executable(detector src/executables/detector.cpp)
target_link_libraries(detector DetectorNode)

add_executable(imgtest src/executables/imgtest.cpp)
ament_target_dependencies(imgtest
rclcpp std_msgs sensor_msgs cv_bridge OpenCV ncnn
)

install(TARGETS
   imgtest
   DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
   yolov7_time_test
   DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
   detector
   DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  server_executor
  client_executor
  EXPORT export_${PROJECT_NAME}
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

install(DIRECTORY
  weights
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
