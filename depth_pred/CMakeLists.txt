cmake_minimum_required(VERSION 2.8)

project(depth_pred)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "--std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -msse -msse2 -msse3")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge)

find_package(OpenCV REQUIRED)
find_package(Eigen3)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(../vins_estimator/src/utility include)
include_directories(../camera_models/include)

add_executable(depth_pred_node src/depth_pred_node.cpp include/depth_pred_node.h)
target_link_libraries(depth_pred_node ${catkin_LIBRARIES})


