/*******************************************************
 * Copyright (C) 2022
 * Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
 * Department of Electronic Engineering, Tsinghua University.
 * Created by Zihan Lin on 2022/3/8.
*******************************************************/

#ifndef VINS_GT_VISUALIZATION_H
#define VINS_GT_VISUALIZATION_H

#include <queue>
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

extern ros::Publisher pub_gt_path;
extern nav_msgs::Path gt_path;
extern std::queue<nav_msgs::OdometryConstPtr> gt_odom_buf;

void pubGTPath(ros::Time simT);

void registerGTPub(ros::NodeHandle &n);

#endif //VINS_GT_VISUALIZATION_H
