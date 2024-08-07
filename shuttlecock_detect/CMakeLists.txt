cmake_minimum_required(VERSION 3.8)
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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs)
find_package(ament_cmake_python REQUIRED)
find_package(ncnn REQUIRED)


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


include_directories(include ${ament_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Yolov7
  src/helpers/Yolov7.cpp
)
target_link_libraries(Yolov7 ncnn)

ament_target_dependencies(Yolov7
ncnn 
)

add_library(DetectorNode
  src/nodes/DetectorNode.cpp 
)
ament_target_dependencies(DetectorNode
rclcpp std_msgs sensor_msgs cv_bridge OpenCV ncnn
)
target_link_libraries(DetectorNode Yolov7)

add_executable(yolov7_time_test src/executables/yolov7_time_test.cpp)
target_link_libraries(yolov7_time_test Yolov7 ncnn)
ament_target_dependencies(yolov7_time_test OpenCV ncnn)

add_executable(detector src/executables/detector.cpp)
target_link_libraries(detector DetectorNode)

install(TARGETS
   yolov7_time_test
   DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
   detector
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

ament_package()
