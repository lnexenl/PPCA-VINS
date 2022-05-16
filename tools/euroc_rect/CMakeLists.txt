cmake_minimum_required(VERSION 3.22)
project(undistort)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV)
find_package(rosbag)
find_package(sensor_msgs)
find_package(cv_bridge)

include_directories(${OpenCV_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS})

add_executable(euroc_rect src/euroc_rect.cpp)

target_link_libraries(euroc_rect ${OpenCV_LIBS} ${rosbag_LIBRARIES} ${sensor_msgs_LIBRARIES} ${cv_bridge_LIBRARIES})
