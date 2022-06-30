/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/4/8.
*******************************************************/

#ifndef VINS_PRIOR_OPT_H
#define VINS_PRIOR_OPT_H
#include <vector>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
template <class T>
using vector = std::vector<T>;
using posemap = std::map<double, vector<float>>;
using namespace Eigen;

class priorOpt
{
public:
    priorOpt() {
        threadOpt = std::thread(&priorOpt::optimize, this);
    };
    ~priorOpt() = default;
    void priorLocatingInput(ros::Time t, geometry_msgs::PoseWithCovarianceStamped p);
    void vioOdomInput(ros::Time tt, Eigen::Vector3f p, Eigen::Quaternionf q);
    void getGlobalOdom(Eigen::Vector3f& p, Eigen::Quaternionf& q);
    nav_msgs::Path globalPath;
    ros::Publisher globalPathPub;
    std::string resultDir;
    int run_cnt = 1;

private:
    void optimize();
    void updateGlobalPath();
    double t = 0.0;
    bool newPriorLocating = false;
    posemap vioPoseMap, globalPoseMap, priorPoseMap;
    std::mutex mVioPoseMap;
    Eigen::Matrix4f PRIOR_T_VIO = Eigen::Matrix4f::Identity();
    Eigen::Vector3f lastP;
    Eigen::Quaternionf lastQ;
    std::thread threadOpt;
};


#endif //VINS_PRIOR_OPT_H
