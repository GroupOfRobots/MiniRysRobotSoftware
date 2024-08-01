cmake_minimum_required(VERSION 3.10)

project(YOLOv7TimeTest)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(YOLOv7TimeTest yolov7_time_test.cpp yolo.cpp yolo.h)

# Znalezienie pakietów OpenCV
find_package(OpenCV REQUIRED)

# Znalezienie bibliotek ncnn (przykładowo, musisz dostosować do swojego środowiska)
#find_package(ncnn REQUIRED)

# Dodanie plików nagłówkowych
include_directories(${OpenCV_INCLUDE_DIRS})

set(ncnn_DIR /home/Jakub/ncnn/build/install/lib/cmake/ncnn )
find_package(ncnn REQUIRED)
# (Opcjonalnie) Dodanie katalogów z plikami nagłówkowymi dla innych bibliotek, jeśli są wymagane


# Dodanie bibliotek OpenCV do projektu
target_link_libraries(YOLOv7TimeTest ${OpenCV_LIBS} ncnn )
