/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/4/8.
*******************************************************/
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include "prior_opt.h"

priorOpt priorOptimizer;
std::queue<geometry_msgs::PoseWithCovarianceStampedConstPtr> priorQueue;
nav_msgs::Path* globalPath = nullptr;
nav_msgs::Path vioPath;
std::mutex m_buf;

int prior_id = 0;

void vio_cb(const nav_msgs::OdometryConstPtr& msg) {
    double lastT = 0.0;
    Vector3f tmpT(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Quaternionf tmpQ(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    priorOptimizer.vioOdomInput(msg->header.stamp, tmpT, tmpQ);

    Vector3f globalT;
    Quaternionf globalQ;
    priorOptimizer.getGlobalOdom(globalT, globalQ);

    nav_msgs::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = "world";
    odom.header.frame_id = "world";
    odom.pose.pose.position.x = globalT.x();
    odom.pose.pose.position.y = globalT.y();
    odom.pose.pose.position.z = globalT.z();
    odom.pose.pose.orientation.w = globalQ.w();
    odom.pose.pose.orientation.x = globalQ.x();
    odom.pose.pose.orientation.y = globalQ.y();
    odom.pose.pose.orientation.z = globalQ.z();
}

void prior_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    m_buf.lock();
    priorOptimizer.priorLocatingInput(msg->header.stamp, *msg.get());
    m_buf.unlock();
}

void viopath_cb(const nav_msgs::PathConstPtr msg) {
    vioPath = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "prior_opt_node");
    std::string confFile = argv[1];
    cv::FileStorage fsSettings(confFile, cv::FileStorage::READ);
    fsSettings["output_path"] >> priorOptimizer.resultDir;
    ros::NodeHandle nh("~");
    if (!boost::filesystem::exists(priorOptimizer.resultDir)) {
        boost::filesystem::create_directories(priorOptimizer.resultDir);
    }
    for (;; priorOptimizer.run_cnt++) {
        if (!boost::filesystem::exists(priorOptimizer.resultDir + "/" + std::to_string(priorOptimizer.run_cnt) + "/prior.txt"))
            break;
    }
    ROS_INFO("result path: %s", (priorOptimizer.resultDir + "/" + std::to_string(priorOptimizer.run_cnt) + "/prior.txt").c_str());
    globalPath = &priorOptimizer.globalPath;

    ros::Subscriber vioSub = nh.subscribe("/vins_estimator/odometry", 100, vio_cb);
    ros::Subscriber priorSub = nh.subscribe("/prior_locate_node/reg_pose", 100, prior_cb);
    ros::Subscriber vioPathSub = nh.subscribe("/vins_estimator/path", 100, viopath_cb);

    priorOptimizer.globalPathPub = nh.advertise<nav_msgs::Path>("/prior_opt_node/global_path", 100);
    ros::spin();
    return 0;
}