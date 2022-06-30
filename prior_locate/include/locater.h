/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/3/14.
*******************************************************/

#ifndef VINS_LOCATER_H
#define VINS_LOCATER_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/persistence.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/bind.hpp>

#include <queue>
#include <mutex>
#include <chrono>
#include <thread>
#include <string>
#include "frames.h"
#include "../third_party/libSGM/include/libsgm.h"

using namespace pcl_conversions;

class Locater
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ros::Publisher depth_img0_pub, depth_img1_pub, depth_pub, local_pc_pub, filtered_pc_pub, prior_locate_pub;
    ros::Time dkf_stamp, filter_stamp;
    std::string resultDir, IMG0_TOPIC, IMG1_TOPIC;

    Locater(std::string confFile);

    void loadYAML(std::string confFile);

    void depth_vis();

    void update_depth(sgm::StereoSGM& s);

    void insert_depthKF();

    void temporal_filter();

    void align();

    void start(sgm::StereoSGM& s);

    void img0_callback(const sensor_msgs::ImageConstPtr& msg);

    void img1_callback(const sensor_msgs::ImageConstPtr& msg);

    void vio_callback(const nav_msgs::Path& msg);
    int WIDTH, HEIGHT;

private:
    float FX, FY, CX, CY, BASELINE_LEN, DEPTH_TH, MAX_DEPTH, FIT_TH, TRANS_TH, VAR_FACTOR;
    int VAL_FRAME;
    bool newNDT = false;
    std::string pc_file;
    std::mutex mDepth, mVio, mImg0, mImg1, mDKF;
    nav_msgs::Path vioPath;
    Eigen::Vector3d clip_range = Eigen::Vector3d::Zero();
    fast_gicp::FastGICP<PointXYZ, PointXYZ> fgicp;
    Vector3f init_tran;
    Quaternionf init_quat;
    PointCloudXYZ::Ptr priorPCPtr;
    Eigen::Matrix3f RIC, RIC_inv;
    Eigen::Vector3f TIC;
    std::queue<sensor_msgs::ImagePtr> depthBuf;
    std::queue<sensor_msgs::ImageConstPtr> img0, img1;
    std::deque<dKeyframe> dkf;
    ndtFrame ndtf;
    pcl::VoxelGrid<PointXYZ> vg;
    std::thread t1, t2, t3, t4;
};
#endif //VINS_LOCATER_H
