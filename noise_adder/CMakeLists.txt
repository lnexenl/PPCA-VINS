cmake_minimum_required(VERSION 3.9)

project(noise_adder CXX)


set(CMAKE_PROJECT_VERSION 0.1.0)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -march=native -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -Wall -ggdb")
include(CheckLanguage)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        )

find_package(OpenCV REQUIRED COMPONENTS core)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})

include_directories(./include/)

add_executable(noise_adder_node src/noise_adder_node.cpp)
target_link_libraries(noise_adder_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

