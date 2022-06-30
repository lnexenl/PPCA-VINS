/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/3/25.
*******************************************************/

#ifndef VINS_FRAMES_H
#define VINS_FRAMES_H
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/filesystem.hpp>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/common.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <deque>
#include <iterator>
#include <sstream>
using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;
using namespace Eigen;
class dKeyframe;
extern std::string resultDir;
extern ros::Publisher transformd_pc_pub;

class ndtFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ndtFrame() = default;
    ;
    //    ~ndtFrame() {
    //        if (false) {
    //            localCloud.header.stamp = stamp.toNSec();
    //            //            cv::Mat outMat(filteredDepth.rows(), filteredDepth.cols(), CV_32FC1);
    //            //            cv::eigen2cv(filteredDepth, outMat);
    //            std::stringstream file;
    //            //            ros::NodeHandle nh("~");
    //            //            std::string resultDir;
    //            //            nh.getParam("resultDir", resultDir);
    //            file << resultDir << "/" << stamp.toNSec() << ".pcd";
    //            //            cv::imwrite(file.str(), outMat);
    //            //            Matrix4f T = Matrix4f::Identity();
    //            //            T.block<3, 3>(0, 0) = R[2].inverse();
    //            //            T.block<3, 1>(0, 3) = -R[2].inverse() * vio_T;
    //            //            PointCloudXYZ tmpPC;
    //            //            pcl::transformPointCloud(localCloud, tmpPC, T);
    //            pcl::io::savePCDFile(file.str(), localCloud, true);
    // //            localCloud.header.stamp = 1;
    // //            localCloud.header.frame_id = "world";
    // //            sensor_msgs::PointCloud2 rospc2;
    // //            pcl::toROSMsg(localCloud, rospc2);
    // //            rospc2.header.stamp = ros::Time::now();
    // //            transformd_pc_pub.publish(rospc2);
    //        }

    //       };
    void addFrame(const dKeyframe& dkf0);
    void buildVioLocalCloud();
    std::vector<Matrix3f> R;
    std::vector<Vector3f> T;
    Matrix4f registerToPriorMap(pcl::VoxelGrid<PointXYZ>& vg, pcl::Registration<PointXYZ, PointXYZ>& ndt, const PointCloudXYZ::ConstPtr& priorPtr, Eigen::Vector3d& clip_range, Matrix4f& m);
    std::vector<PointCloudXYZ> pointcloud;
    PointCloudXYZ localCloud;
    ros::Time stamp;

private:
};

class dKeyframe
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointCloudXYZ::Ptr worldPC;
    MatrixXf filteredDepth, depth, projected_p;
    MatrixXi cnt;
    ros::Time stamp = ros::Time(0, 0);
    dKeyframe(int width, int height, float fx, float fy, float cx, float cy, float depth_th, int val_frames, float max_depth, float baseline, std::string resultDir) {
        worldPC.reset((new PointCloudXYZ()));
        worldPC->resize(width * height);
        cnt.resize(height, width);
        projected_p.resize(3, width * height);
        cnt.setZero();
        this->width = width;
        this->height = height;
        this->cx = cx;
        this->cy = cy;
        this->fx = fx;
        this->fy = fy;
        this->depth_th = depth_th;
        this->val_frames = val_frames;
        this->max_depth = max_depth;
        this->baseline = baseline;
        this->resultDir = resultDir;
    };
    ~dKeyframe() {
        cv::Mat disp, cvcnt, mask;
        cv::eigen2cv(filteredDepth, disp);
        cv::eigen2cv(cnt, cvcnt);
        cv::bitwise_or(cvcnt<val_frames, disp> max_depth, mask);
        disp.setTo(baseline * fx, mask);
        disp = baseline * fx / disp;
        disp.convertTo(disp, CV_16UC1, 256);
        std::stringstream file;
        file << resultDir << stamp.toNSec() << ".png";
        cv::imwrite(file.str(), disp);
    }
    PointCloudXYZ filteredPC;
    void depth2world(const sensor_msgs::ImageConstPtr& depthptr, const Matrix3f& RIC, const Vector3f& TIC);
    void proj2KF(dKeyframe& kf0, Matrix3f& R, Vector3f& T);
    void setVioRT(int ind, nav_msgs::Path& p);
    void setVioRT(ros::Time stamp, nav_msgs::Path& p);
    void getVioRT(Matrix3f& R, Vector3f& T);
    sensor_msgs::PointCloud2 visualize(const Matrix3f& RIC, const Vector3f& TIC);
    void updateCnt(const Eigen::MatrixXf& pts);
    void genFilteredCloud(const Matrix3f& RIC, const Vector3f& TIC);

private:
    friend class ndtFrame;
    std::string resultDir, dispDir;
    Matrix3f vio_R, vio_R_inv;
    Vector3f vio_T;
    uint64_t id;
    float fx, fy, cx, cy, depth_th, max_depth, baseline;
    int val_frames;
    int width, height;
};

#endif //VINS_FRAMES_H
