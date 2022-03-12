/*******************************************************
 * Copyright (C) 2022
 * Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
 * Department of Electronic Engineering, Tsinghua University.
 * Created by Zihan Lin on 2022/3/12.
*******************************************************/
#ifndef DEPTH_PRED
#define DEPTH_PRED

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>

#include <queue>
#include <mutex>

#include "visualization.h"
#include "estimator.h"
#endif

std::queue<sensor_msgs::ImageConstPtr> depthBuf;
std::mutex mDepth;
extern nav_msgs::Path path;
extern Estimator estimator;

void depth_callback(const sensor_msgs::ImageConstPtr &msg) {
    mDepth.lock();
    depthBuf.push(msg);
    mDepth.unlock();
    if(path.poses.size() >= 3) {

    }
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_pred_node");
    ros::NodeHandle n("~");

    ros::Subscriber depth_sub = n.subscribe("/airsim_node/car/left/depth", 10, depth_callback);

    return 0;
}