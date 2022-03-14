/*******************************************************
 * Copyright (C) 2022
 * Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
 * Department of Electronic Engineering, Tsinghua University.
 * Created by Zihan Lin on 2022/3/12.
*******************************************************/

#include "depth_pred_node.h"

std::queue<sensor_msgs::ImageConstPtr> depthBuf;
std::mutex mDepth, mVio, mImg0, mImg1;
std::queue<nav_msgs::PathConstPtr> pathBuf;
std::queue<sensor_msgs::ImageConstPtr> img0, img1;

ros::Publisher depth_img0_pub, depth_img1_pub;

int cur_kfid = 1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_pred_node");
    ros::NodeHandle n("~");

    if (argc != 2)
        return 0;

    string conf_file = argv[1];
    printf("config file: %s.\n", argv[1]);

    cv::FileStorage fsSettings(conf_file, cv::FileStorage::READ);
    cv::Mat cv_T0, cv_T1;
    fsSettings["body_T_cam0"] >> cv_T0;
    fsSettings["body_T_cam1"] >> cv_T1;
    Eigen::Matrix4d T0, T1;
    cv::cv2eigen(cv_T0, T0);
    cv::cv2eigen(cv_T1, T1);


    ros::Subscriber depth_sub = n.subscribe("/airsim_node/car/left/depth", 10, depth_callback);
    ros::Subscriber vio_sub = n.subscribe("/vins_estimator/path", 2, vio_callback);
    ros::Subscriber img0_sub = n.subscribe("/airsim_node/car/left/Scene_450p", 10, img0_callback);
    ros::Subscriber img1_sub = n.subscribe("/airsim_node/car/left/Scene_450p", 10, img1_callback);

    depth_img0_pub = n.advertise<sensor_msgs::Image>("/depth_img/left", 5);
    depth_img1_pub = n.advertise<sensor_msgs::Image>("/depth_img/right", 5);

//    ros::spin();
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}