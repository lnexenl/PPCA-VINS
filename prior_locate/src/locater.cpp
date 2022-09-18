/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/5/18.
*******************************************************/

#include "locater.h"
Locater::Locater(std::string confFile) {
    fgicp.setNumThreads(8);
    //        sgbm = cv::StereoSGBM::create(0, 160, 7, 0, 0, -1, 0, 15, 20, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
    priorPCPtr.reset(new PointCloudXYZ());
    loadYAML(confFile);
    if (pcl::io::loadPCDFile(pc_file, *priorPCPtr)) {
        ROS_ERROR("ERROR WHEN LOADING PRIOR MAP");
        exit(0);
    }
    else {
        ROS_INFO("LOAD POINT CLOUD FROM %s, %d points", pc_file.c_str(), priorPCPtr->size());
    }
}
void Locater::loadYAML(std::string confFile) {
    cv::FileStorage fsSettings(confFile, cv::FileStorage::READ);
    cv::Mat cv_T0, cv_T1;
    fsSettings["body_T_cam0"] >> cv_T0;
    fsSettings["body_T_cam1"] >> cv_T1;
    fsSettings["image0_topic"] >> IMG0_TOPIC;
    fsSettings["image1_topic"] >> IMG1_TOPIC;
    Eigen::Matrix4d T0, T1;
    cv::cv2eigen(cv_T0, T0);
    cv::cv2eigen(cv_T1, T1);
    RIC = T0.block<3, 3>(0, 0).cast<float>();
    RIC_inv = T0.block<3, 3>(0, 0).inverse().cast<float>();
    TIC = T0.block<3, 1>(0, 3).cast<float>();

    fsSettings["image_width"] >> WIDTH;
    fsSettings["image_height"] >> HEIGHT;
    fsSettings["baseline_length"] >> BASELINE_LEN;
    fsSettings["prior_locate"]["depth_threshold"] >> DEPTH_TH;
    fsSettings["prior_locate"]["validate_frames"] >> VAL_FRAME;
    fsSettings["prior_locate"]["max_depth"] >> MAX_DEPTH;
    fsSettings["prior_locate"]["fitness_threshold"] >> FIT_TH;
    fsSettings["prior_locate"]["reg_trans_threshold"] >> TRANS_TH;
    fsSettings["prior_locate"]["var_factor"] >> VAR_FACTOR;
    fsSettings["prior_locate"]["pointcloud"] >> pc_file;

    fsSettings["prior_locate"]["clip_range"]["x"] >> clip_range[0];
    fsSettings["prior_locate"]["clip_range"]["y"] >> clip_range[1];
    fsSettings["prior_locate"]["clip_range"]["z"] >> clip_range[2];
    ROS_INFO("Clip range: %f, %f, %f", clip_range.x(), clip_range.y(), clip_range.z());
    float vg_size = fsSettings["prior_locate"]["voxel_grid_size"];
    vg.setLeafSize(vg_size, vg_size, vg_size);

    cv::Mat tran, quat;
    fsSettings["prior_locate"]["init_tran"] >> tran;
    fsSettings["prior_locate"]["init_quat"] >> quat;
    init_tran = Vector3f(tran.at<float>(0, 0), tran.at<float>(0, 1), tran.at<float>(0, 2));
    init_quat = Quaternionf(quat.at<float>(0, 0), quat.at<float>(0, 1), quat.at<float>(0, 2), quat.at<float>(0, 3));
    ROS_INFO("init_tran: %f, %f, %f", init_tran.x(), init_tran.y(), init_tran.z());
    ROS_INFO("init_quat: %f, %f, %f, %f", init_quat.w(), init_quat.x(), init_quat.y(), init_quat.z());

    int pn = confFile.find_last_of('/');
    std::string configPath = confFile.substr(0, pn);

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    ROS_WARN("%s", cam0Path.c_str());
    cv::FileStorage camSettings(cam0Path, cv::FileStorage::READ);
    camSettings["projection_parameters"]["fx"] >> FX;
    camSettings["projection_parameters"]["fy"] >> FY;
    camSettings["projection_parameters"]["cx"] >> CX;
    camSettings["projection_parameters"]["cy"] >> CY;
    ROS_INFO("Image size: %d x %d, Baseline length: %f", WIDTH, HEIGHT, BASELINE_LEN);
    ROS_INFO("fx:%f, fy:%f, cx:%f, cy:%f", FX, FY, CX, CY);

    fsSettings["output_path"] >> resultDir;
    fsSettings.release();
    camSettings.release();
}
/* void Locater::depth_vis() {
    PointCloudXYZ pclpc;
    sensor_msgs::PointCloud2 pc;
    for (uint64_t i = 0; i < dkf.rbegin()->worldPC->size(); i += 20) {
        if (dkf.rbegin()->depth(int(i / WIDTH), i % WIDTH) < 80 &&
            -2 < dkf.rbegin()->worldPC->points[i].z && dkf.rbegin()->worldPC->points[i].z < 2)
            pclpc.emplace_back(dkf.rbegin()->worldPC->points[i]);
    }
    toROSMsg(pclpc, pc);
    pc.header.frame_id = "world";
    pc.header.stamp = dkf.rbegin()->stamp;
    local_pc_pub.publish(pc);
} */
void Locater::update_depth(sgm::StereoSGM& s) {
    int cur_depthid = 0;
    while (!ros::isShuttingDown()) {
        if (cur_depthid >= vioPath.poses.size()) {
            continue;
        }
        ROS_ASSERT(cur_depthid < vioPath.poses.size());
        while (!img0.empty() && (img0.front()->header.stamp.toSec() < vioPath.poses[cur_depthid].header.stamp.toSec() - 0.003 || img0.front()->header.stamp.toSec() < img1.front()->header.stamp.toSec() - 0.003)) {
            img0.pop();
        }

        while (!img1.empty() && (img1.front()->header.stamp.toSec() < vioPath.poses[cur_depthid].header.stamp.toSec() - 0.003 || img1.front()->header.stamp.toSec() < img0.front()->header.stamp.toSec() - 0.003)) {
            img1.pop();
        }
        if (img0.empty() || img1.empty()) {
            continue;
        }
        //        auto start = std::chrono::steady_clock::now();
        auto cvptr0 = cv_bridge::toCvShare(img0.front(), "mono8");
        auto cvptr1 = cv_bridge::toCvShare(img1.front(), "mono8");

        cv::Mat left_disp(cvptr0->image.size(), CV_16S), depth, disp_map, disp_color;

        s.execute(cvptr0->image.data, cvptr1->image.data, left_disp.data);
        cv::Mat mask = left_disp == s.get_invalid_disparity();
        left_disp.convertTo(depth, CV_32FC1, 1. / sgm::StereoSGM::SUBPIXEL_SCALE);
        depth.convertTo(disp_map, CV_8U, 255. / 128.);
        cv::applyColorMap(disp_map, disp_color, cv::COLORMAP_JET);
        disp_color.setTo(cv::Scalar(0, 0, 0), mask);
        depth.setTo(1, mask);
        depth.setTo(1, depth <= 1);
        depth = FX * BASELINE_LEN / depth;
        cv_bridge::CvImage cvImg;
        cvImg.header = img0.front()->header;
        sensor_msgs::Image img;

        img0.pop();
        img1.pop();
        cur_depthid++;

        cvImg.image = disp_color;
        cvImg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        cvImg.toImageMsg(img);
        depth_pub.publish(img);

        cvImg.image = depth;
        cvImg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        mDepth.lock();
        depthBuf.emplace(cvImg.toImageMsg());
        mDepth.unlock();
    }
}
void Locater::insert_depthKF() {
    while (!ros::isShuttingDown()) {
        if (!depthBuf.empty() && depthBuf.front()->header.stamp.toSec() <= vioPath.poses.rbegin()->header.stamp.toSec() + 0.0001) {
            sensor_msgs::ImagePtr msg = depthBuf.front();
            dkf_stamp = msg->header.stamp;
            dKeyframe tmpdkf(WIDTH, HEIGHT, FX, FY, CX, CY, DEPTH_TH, VAL_FRAME, MAX_DEPTH, BASELINE_LEN, resultDir);
            tmpdkf.setVioRT(msg->header.stamp, vioPath);
            tmpdkf.depth2world(msg, RIC, TIC);
            mDKF.lock();
            dkf.push_back(tmpdkf);
            mDKF.unlock();
            //                depth_vis();
            depthBuf.pop();
        }
    }
}
void Locater::temporal_filter() {
    int ndtCnt = 0;
    while (!ros::isShuttingDown()) {
        // ROS_INFO("DKF SIZE:%zu", dkf.size());
        mDKF.lock();
        if (int(dkf.size()) >= 7) {
            ROS_ASSERT(dkf.size() >= 7);
            if (filter_stamp < dkf[3].stamp) {
                filter_stamp = dkf[3].stamp;
                for (int i = 1; i <= 5; ++i) {
                    if (i == 3)
                        continue;
                    dkf[i].proj2KF(dkf[3], RIC_inv, TIC);
                    dkf[3].updateCnt(dkf[i].projected_p);
                }
                dkf[3].genFilteredCloud(RIC, TIC);
                if (ndtCnt == 5) {
                    ndtCnt = 0;
                    ndtFrame tmpNDTFrame;
                    for (auto it = dkf.begin(); it != std::next(dkf.begin(), 5); ++it) {
                        tmpNDTFrame.addFrame(*it);
                    }
                    tmpNDTFrame.stamp = dkf[1].stamp;
                    ndtf = tmpNDTFrame;
                    newNDT = true;
                }
                filtered_pc_pub.publish(dkf[3].visualize(RIC, TIC));
                dkf.pop_front();
                ndtCnt++;
            }
        }
        mDKF.unlock();
    }
}
void Locater::align() {
    Matrix3f cur_R, rot = Matrix3f::Identity();
    Matrix4f m = Matrix4f::Identity();
    Matrix4f mm = Matrix4f::Identity();
    m.block<3, 1>(0, 3) = init_tran;
    m.block<3, 3>(0, 0) = init_quat.toRotationMatrix();
    while (!ros::isShuttingDown()) {
        if (!ndtf.pointcloud.empty() && newNDT) {
            newNDT = false;
            ndtFrame it(ndtf);
            it.buildVioLocalCloud();
            PointCloudXYZ::Ptr ptr(new PointCloudXYZ(it.localCloud));
            pcl::transformPointCloud(*ptr, *ptr, m);
            it.localCloud = *ptr;
            cur_R = it.R[2];
            Matrix4f tmp_m = it.registerToPriorMap(vg, fgicp, priorPCPtr, clip_range, m);
            double fitness = fgicp.getFitnessScore();

            float ang = Quaternionf(tmp_m.block<3, 3>(0, 0)).angularDistance(Quaternionf(1, 0, 0, 0));
            Quaternionf qm(m.block<3, 3>(0, 0));
            ROS_WARN("%zu %f %f %f\n", it.stamp.toNSec(), tmp_m.coeff(0, 3), tmp_m.coeff(1, 3), tmp_m.coeff(2, 3));
            ROS_INFO("%f %f", fitness, ang);
            ROS_INFO("%1.3f %1.3f %1.3f    %1.3f %1.3f %1.3f %1.3f", m(0, 3), m(1, 3), m(2, 3), qm.w(), qm.x(), qm.y(), qm.z());
            if (fgicp.hasConverged() && fitness < FIT_TH && tmp_m.block<3, 1>(0, 3).norm() < TRANS_TH) {
                //                auto tmp_m = fgicp.getFinalTransformation();

                Matrix<double, 6, 6> cov = (fgicp.getFinalHessian()).inverse();
                double c = sqrt(cov(0, 0) + cov(1, 1) + cov(2, 2)) * VAR_FACTOR;
                ROS_WARN("%2.4f", c);

                m.block<3, 1>(0, 3) += tmp_m.block<3, 1>(0, 3);
                m.block<3, 3>(0, 0) = tmp_m.block<3, 3>(0, 0) * m.block<3, 3>(0, 0);
                rot = tmp_m.block<3, 3>(0, 0) * rot;
                Quaternionf tmpQ = Quaternionf(m.block<3, 3>(0, 0) * cur_R);
                //                    ROS_INFO("%f %f %f", tmpQ.w(), tmpQ.x(), tmpQ.y(), tmpQ.z());
                //                    ROS_INFO("%f %f %f", 100000*cov(0, 0), 100000*cov(1, 1), 100000*cov(2, 2));
                Quaternionf rotq(rot * cur_R);
                //                    m = tmp_m;
                geometry_msgs::PoseWithCovarianceStamped tmpPose;
                tmpPose.header.frame_id = "world";
                tmpPose.header.stamp = it.stamp;
                tmpPose.pose.pose.position.x = m(0, 3) - init_tran.x() + it.T[2](0);
                tmpPose.pose.pose.position.y = m(1, 3) - init_tran.y() + it.T[2](1);
                tmpPose.pose.pose.position.z = m(2, 3) - init_tran.z() + it.T[2](2);
                tmpPose.pose.pose.orientation.w = rotq.w();
                tmpPose.pose.pose.orientation.x = rotq.x();
                tmpPose.pose.pose.orientation.y = rotq.y();
                tmpPose.pose.pose.orientation.z = rotq.z();
                tmpPose.pose.covariance[0] = c;

                //                ndt_path.poses.emplace_back(tmpPose);
                prior_locate_pub.publish(tmpPose);
            }
        }
    }
}
void Locater::start(sgm::StereoSGM& s) {
    t1 = std::thread(&Locater::update_depth, this, std::ref(s));
    ROS_INFO("Thread 1 started");
    t2 = std::thread(&Locater::insert_depthKF, this);
    ROS_INFO("Thread 2 started");
    t3 = std::thread(&Locater::temporal_filter, this);
    ROS_INFO("Thread 3 started");
    t4 = std::thread(&Locater::align, this);
    ROS_INFO("Thread 4 started");
}
void Locater::img0_callback(const sensor_msgs::ImageConstPtr& msg) {
    mImg0.lock();
    img0.emplace(msg);
    mImg0.unlock();
}
void Locater::img1_callback(const sensor_msgs::ImageConstPtr& msg) {
    mImg1.lock();
    img1.emplace(msg);
    mImg1.unlock();
}
void Locater::vio_callback(const nav_msgs::Path& msg) {
    mVio.lock();
    vioPath = msg;
    mVio.unlock();
}
