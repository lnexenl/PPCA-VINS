/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/3/25.
*******************************************************/
#include "frames.h"
using namespace pcl;

void dKeyframe::setVioRT(int ind, nav_msgs::Path& p) {
    id = ind;
    vio_R = Quaternionf(p.poses[ind].pose.orientation.w,
                        p.poses[ind].pose.orientation.x,
                        p.poses[ind].pose.orientation.y,
                        p.poses[ind].pose.orientation.z)
                .matrix();
    vio_R_inv = vio_R.inverse();
    vio_T = Vector3f(p.poses[ind].pose.position.x,
                     p.poses[ind].pose.position.y,
                     p.poses[ind].pose.position.z);
}
void dKeyframe::setVioRT(ros::Time stamp, nav_msgs::Path& p) {
    int i;
    this->stamp = stamp;
    for (i = p.poses.size() - 1; i >= 0; --i)
        if (abs(p.poses[i].header.stamp.toSec() - this->stamp.toSec()) < 0.0001) {
            setVioRT(i, p);
            break;
        }
}

void dKeyframe::depth2world(const sensor_msgs::ImageConstPtr& depthptr, const Matrix3f& RIC, const Vector3f& TIC) {
    auto cvDepthPtr = cv_bridge::toCvShare(depthptr, depthptr->encoding);
    cv::cv2eigen<float>(cvDepthPtr->image, depth);
    for (auto i = 0; i < height; ++i) {
        for (auto j = 0; j < width; ++j) {
            Vector3f pt = { (float(j) - float(cx)) / float(fx), (float(i) - float(cy)) / float(fy), 1.0 };
            pt = depth.coeff(i, j) * pt;
            pt = vio_R * (RIC * pt + TIC) + vio_T;
            worldPC->points[i * width + j] = PointXYZ(pt.x(), pt.y(), pt.z());
        }
    }
    this->filteredDepth = depth;
}

void dKeyframe::getVioRT(Matrix3f& R, Vector3f& T) {
    R = vio_R;
    T = vio_T;
}

void dKeyframe::proj2KF(dKeyframe& kf0, Matrix3f& RIC_inv, Vector3f& TIC) {
    Matrix3f R0;
    Vector3f T0;
    kf0.getVioRT(R0, T0);
    R0 = R0.inverse().eval();
    for (auto i = 0; i < int(worldPC->size()); ++i) {
        Vector3f pt = RIC_inv * ((R0 * (Vector3f(worldPC->points[i].x, worldPC->points[i].y, worldPC->points[i].z) - T0)) - TIC);
        projected_p.col(i) = pt;
    }
}

void dKeyframe::updateCnt(const Eigen::MatrixXf& pts) {
    /// \var pts:points from other cam frame projected to current cam frame.
    for (auto pt:pts.colwise()) {
        float d = pt(2);
        uint32_t x = floor(pt(0) / d * fx + cx);
        uint32_t y = floor(pt(1) / d * fy + cy);
        //        ROS_INFO("%ld, %ld, %d, %d", depth.cols(), depth.rows(), x, y);
        if (x < 0 || y < 0 || x >= depth.cols() || y >= depth.rows())
            continue;
        else {
            if (abs(depth(y, x) - d) < depth_th) {
                cnt(y, x) += 1;
                filteredDepth(y, x) = (filteredDepth(y, x) * cnt(y, x) + d) / (cnt(y, x) + 1);
            }
        }
    }
}

void dKeyframe::genFilteredCloud(const Matrix3f& RIC, const Vector3f& TIC) {

    //    ROS_INFO("MIN MAX CNT: %d, %d", cnt.minCoeff(), cnt.maxCoeff());
    for (int i = 0; i < filteredDepth.rows(); ++i)
        for (int j = 0; j < filteredDepth.cols(); ++j) {
            if (cnt(i, j) >= val_frames && filteredDepth(i, j) < max_depth) {
                Vector3f pt = { (float(j) - float(cx)) / float(fx), (float(i) - float(cy)) / float(fy), 1.0 };
                pt = filteredDepth.coeff(i, j) * pt;
                pt = vio_R * (RIC * pt + TIC) + vio_T;
                filteredPC.push_back({ pt.x(), pt.y(), pt.z() });
            }
        }
    filteredPC.header.stamp = stamp.toNSec();
    //        ROS_INFO("point cloud points: %zu", filteredPC.points.size());
}

sensor_msgs::PointCloud2 dKeyframe::visualize(const Matrix3f& RIC, const Vector3f& TIC) {
    PointCloudXYZ pclpc;
    sensor_msgs::PointCloud2 pc;
    for (auto i = 0; i < filteredDepth.rows(); i += 5)
        for (auto j = 0; j < filteredDepth.cols(); j += 5) {
            if (cnt(i, j) >= val_frames && filteredDepth(i, j) < max_depth) {
                Vector3f pt = { (float(j) - float(cx)) / float(fx), (float(i) - float(cy)) / float(fy), 1.0 };
                pt = filteredDepth.coeff(i, j) * pt;
                pt = vio_R * (RIC * pt + TIC) + vio_T;
                pclpc.push_back(PointXYZ(pt.x(), pt.y(), pt.z()));
            }
        }
    this->filteredPC = pclpc;
    toROSMsg(pclpc, pc);
    pc.header.frame_id = "world";
    pc.header.stamp = stamp;
    return pc;
}