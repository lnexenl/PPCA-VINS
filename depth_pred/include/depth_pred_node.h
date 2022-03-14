/*******************************************************
 * Copyright (C) 2022
 * Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
 * Department of Electronic Engineering, Tsinghua University.
 * Created by Zihan Lin on 2022/3/14.
*******************************************************/

#ifndef VINS_DEPTH_PRED_NODE_H
#define VINS_DEPTH_PRED_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>

#include <opencv2/core/persistence.hpp>

#include <queue>
#include <mutex>

#include "visualization.h"

#endif //VINS_DEPTH_PRED_NODE_H

extern std::queue<sensor_msgs::ImageConstPtr> depthBuf;
extern std::mutex mDepth, mVio, mImg0, mImg1;
extern std::queue<nav_msgs::PathConstPtr> pathBuf;
extern std::queue<sensor_msgs::ImageConstPtr> img0, img1;
extern ros::Publisher depth_img0_pub, depth_img1_pub;
extern int cur_kfid;
//extern Estimator estimator;

void img0_callback(const sensor_msgs::ImageConstPtr &msg) {
    mImg0.lock();
    img0.emplace(msg);
    mImg0.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &msg) {
    mImg1.lock();
    img1.emplace(msg);
    mImg1.unlock();
}

void depth_callback(const sensor_msgs::ImageConstPtr &msg) {
    mDepth.lock();
    depthBuf.emplace(msg);
    mDepth.unlock();
    if (pathBuf.front()->poses.size() >= 3 && depthBuf.size() >= 3) {

    }
    return;
}

void vio_callback(const nav_msgs::PathConstPtr &msg) {
    mVio.lock();
    if (pathBuf.size())
        pathBuf.pop();
    pathBuf.push(msg);
    mVio.unlock();
    if(pathBuf.front()->poses.size() >= 3) {
        mImg0.lock();
        while(pathBuf.front()->poses[cur_kfid].header.stamp > img0.front()->header.stamp && !img0.empty())
            img0.pop();
        depth_img0_pub.publish(img0.front());
        img0.pop();
        mImg0.unlock();

        mImg1.lock();
        while(pathBuf.front()->poses[cur_kfid].header.stamp > img1.front()->header.stamp && !img1.empty())
            img1.pop();
        depth_img1_pub.publish(img1.front());
        cur_kfid++;
        img1.pop();
        mImg1.unlock();

    }
}
