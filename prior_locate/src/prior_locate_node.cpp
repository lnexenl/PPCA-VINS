/*******************************************************
 * Copyright (C) 2022
 * Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
 * Department of Electronic Engineering, Tsinghua University.
 * Created by Zihan Lin on 2022/3/12.
*******************************************************/

#include "locater.h"

//std::queue<sensor_msgs::ImageConstPtr> img0, img1;

Eigen::Vector3f TIC;
std::string resultDir;
ros::Publisher transformd_pc_pub;

int main(int argc, char** argv) {

    ros::init(argc, argv, "prior_locate_node");
    ros::NodeHandle n("~");
    std::string priorMapFile, confFile;

    PointCloudXYZ::Ptr PCPtr(new PointCloudXYZ);
    //    n.getParam("priorMapFile", priorMapFile);
    n.getParam("confFile", confFile);
    //    priorMapFile = "/home/lnex/KITTI/dataset/kitti_sequence_00.pcd";
    confFile = argv[1];
    Locater locater(confFile);
    resultDir = locater.resultDir;
    for (int i = 1;; i++) {
        if (!boost::filesystem::exists(locater.resultDir + "/" + std::to_string(i) + "/disparity")) {
            boost::filesystem::create_directories(locater.resultDir + "/" + std::to_string(i) + "/disparity");
            locater.resultDir = locater.resultDir + "/" + std::to_string(i) + "/disparity/";
            break;
        }
    }

    ros::Subscriber vio_sub = n.subscribe("/vins_estimator/path", 2, &Locater::vio_callback, &locater);
    ros::Subscriber img0_sub = n.subscribe(locater.IMG0_TOPIC, 10, &Locater::img0_callback, &locater);
    ros::Subscriber img1_sub = n.subscribe(locater.IMG1_TOPIC, 10, &Locater::img1_callback, &locater);

    locater.depth_pub = n.advertise<sensor_msgs::Image>("/prior_locate_node/depth", 1);
    locater.local_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/prior_locate_node/local_pc", 2);
    locater.filtered_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/prior_locate_node/filtered_pc", 1);
    locater.prior_locate_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/prior_locate_node/reg_pose", 1);
    transformd_pc_pub = n.advertise<sensor_msgs::PointCloud2>("/transformed_pc", 1);
    ros::MultiThreadedSpinner spinner(4);

    sgm::StereoSGM::Parameters SGMParam(3, 40, 0.95, true, sgm::PathType::SCAN_8PATH, 0, 1);
    sgm::StereoSGM ssgm(locater.WIDTH, locater.HEIGHT, 128, 8, 16, sgm::EXECUTE_INOUT_HOST2HOST, SGMParam);

    locater.start(ssgm);
    spinner.spin();

    return 0;
}