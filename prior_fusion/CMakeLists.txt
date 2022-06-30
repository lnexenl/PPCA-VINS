cmake_minimum_required(VERSION 2.8)

project(prior_fusion CXX)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_PROJECT_VERSION 0.1.0)
set(CMAKE_CXX_FLAGS "--std=c++17")
set(CMAKE_CXX_FLAGS " -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wno-deprecated-declarations -g")

include(CheckLanguage)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs)
find_package(OpenCV REQUIRED core)
find_package(Ceres REQUIRED)
find_package(Eigen3)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
include_directories(./include/)


add_executable(prior_fusion_node src/prior_opt.cpp src/prior_opt_node.cpp)
target_link_libraries(prior_fusion_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})


