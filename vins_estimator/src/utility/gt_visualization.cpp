/*******************************************************
 * Copyright (C) 2022
 * Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
 * Department of Electronic Engineering, Tsinghua University.
 * Created by Zihan Lin on 2022/3/8.
*******************************************************/

#include "gt_visualization.h"

ros::Publisher pub_gt_path;
nav_msgs::Path gt_path;
std::queue<nav_msgs::OdometryConstPtr> gt_odom_buf;

void registerGTPub(ros::NodeHandle &n) {
    pub_gt_path = n.advertise<nav_msgs::Path>("gt_path", 1000);
}


void pubGTPath(ros::Time simT) {
    nav_msgs::OdometryConstPtr it;
//    ROS_WARN("GT_ODOM_LENGTH: %d", gt_odom_buf.size());
    while (!gt_odom_buf.empty()) {
        it = gt_odom_buf.front();

//        ROS_WARN("simT:%lf, odom_stamp:%lf", simT.toSec(), it->header.stamp.toSec());
        if (it->header.stamp <= simT) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = it->header;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = it->pose.pose.position.x;
            pose_stamped.pose.position.y = -it->pose.pose.position.y;
            pose_stamped.pose.position.z = -it->pose.pose.position.z;
            gt_path.header = it->header;
            gt_path.header.frame_id = "world";
            gt_path.poses.push_back(pose_stamped);
            gt_odom_buf.pop();
        } else
            break;
    }
    pub_gt_path.publish(gt_path);
}