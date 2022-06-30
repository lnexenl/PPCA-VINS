cmake_minimum_required(VERSION 3.9)

project(prior_locate CXX CUDA)

add_subdirectory(third_party/libSGM)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_PROJECT_VERSION 0.1.0)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -march=native -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3  -g -Wall")

#set(CMAKE_CUDA_ARCHITECTURES "86")
set(CUDA_SEPARABLE_COMPILATION ON)

include(CheckLanguage)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        fast_gicp
        )

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(Eigen3)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Boost COMPONENTS system)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIR} ${Boost_INCLUDE_DIR})

include_directories(./include/)

#add_library(sgm STATIC third_party/sgm/disparity_method.cuh third_party/sgm/median_filter.cu third_party/sgm/hamming_cost.cu third_party/sgm/disparity_method.cu third_party/sgm/debug.cu third_party/sgm/costs.cu)
#target_compile_features(sgm PUBLIC cxx_std_17)
#set_property(TARGET sgm PROPERTY CUDA_ARCHITECTURES "86")
#set_property(TARGET sgm PROPERTY CUDA_SEPARABLE_COMPILATION ON)

file(GLOB SRC_FILES src/*.cpp)
add_executable(prior_locate_node ${SRC_FILES})
target_link_libraries(prior_locate_node sgm ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})


