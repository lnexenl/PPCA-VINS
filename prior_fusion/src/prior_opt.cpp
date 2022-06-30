/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/4/8.
*******************************************************/

#include <fstream>
#include "prior_opt.h"
#include "factors.h"

void priorOpt::vioOdomInput(ros::Time tt, Eigen::Vector3f p, Eigen::Quaternionf q) {
    mVioPoseMap.lock();
    vioPoseMap[tt.toSec()] = vector<float>{ p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z() };

    Eigen::Quaternionf globalQ;
    globalQ = PRIOR_T_VIO.block<3, 3>(0, 0) * q;
    Eigen::Vector3f globalP = PRIOR_T_VIO.block<3, 3>(0, 0) * p + PRIOR_T_VIO.block<3, 1>(0, 3);
    vector<float> globalPose{ globalP.x(), globalP.y(), globalP.z(), globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z() };
    globalPoseMap[tt.toSec()] = globalPose;
    lastP = globalP;
    lastQ = globalQ;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = tt;
    pose.header.frame_id = "world";
    pose.pose.position.x = lastP.x();
    pose.pose.position.y = lastP.y();
    pose.pose.position.z = lastP.z();
    pose.pose.orientation.x = lastQ.x();
    pose.pose.orientation.y = lastQ.y();
    pose.pose.orientation.z = lastQ.z();
    pose.pose.orientation.w = lastQ.w();
    globalPath.poses.emplace_back(pose);
    globalPath.header = pose.header;

    mVioPoseMap.unlock();
    newPriorLocating = true;
}

void priorOpt::getGlobalOdom(Eigen::Vector3f& p, Eigen::Quaternionf& q) {
    p = lastP;
    q = lastQ;
}

void priorOpt::priorLocatingInput(ros::Time tt, geometry_msgs::PoseWithCovarianceStamped p) {
    priorPoseMap[tt.toSec()] = vector<float>{ static_cast<float>(p.pose.pose.position.x),
                                              static_cast<float>(p.pose.pose.position.y),
                                              static_cast<float>(p.pose.pose.position.z),
                                              static_cast<float>(p.pose.pose.orientation.w),
                                              static_cast<float>(p.pose.pose.orientation.x),
                                              static_cast<float>(p.pose.pose.orientation.y),
                                              static_cast<float>(p.pose.pose.orientation.z),
                                              static_cast<float>(p.pose.covariance[0]) };
    newPriorLocating = true;
}

void priorOpt::optimize() {
    while (!ros::isShuttingDown()) {

        if (newPriorLocating) {
            ROS_INFO("OPTIMIZING...");
            newPriorLocating = false;
            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction* lossFunction = new ceres::HuberLoss(1.0);
#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1 // Switch to manifold interface when ceres version >= 2.1.0
            ceres::Manifold* localQManifold = new ceres::QuaternionManifold();
            ceres::Manifold* localEManifold = new ceres::EuclideanManifold<3>();
#elif
            ceres::LocalParameterization* localParameterization = new ceres::QuaternionParameterization();
#endif

            mVioPoseMap.lock();
            uint64_t length = vioPoseMap.size();
            double tArray[length][3];
            double qArray[length][4];
            auto iter = globalPoseMap.begin();
            for (auto i = 0; i < length; ++i, ++iter) {
                tArray[i][0] = iter->second[0];
                tArray[i][1] = iter->second[1];
                tArray[i][2] = iter->second[2];
                qArray[i][0] = iter->second[3];
                qArray[i][1] = iter->second[4];
                qArray[i][2] = iter->second[5];
                qArray[i][3] = iter->second[6];
#if CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1
                problem.AddParameterBlock(qArray[i], 4, localQManifold);
                problem.AddParameterBlock(tArray[i], 3, localEManifold);
#elif
                problem.AddParameterBlock(qArray[i], 4, localParameterization);
                problem.AddParameterBlock(tArray[i], 3);
#endif
            }

            posemap::iterator iterVio, iterVioNext, iterPrior, iterPriorinit;
            bool priorinit = false;
            int i = 0, initind = 0;
            for (iterVio = vioPoseMap.begin(); iterVio != vioPoseMap.end(); ++iterVio, ++i) {
                iterVioNext = std::next(iterVio, 1);
                if (iterVioNext != vioPoseMap.end()) {
                    Matrix4f wTi = Matrix4f::Identity(), wTj = Matrix4f::Identity();
                    wTi.block<3, 3>(0, 0) = Quaternionf(iterVio->second[3],
                                                        iterVio->second[4],
                                                        iterVio->second[5],
                                                        iterVio->second[6])
                                                .toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Vector3f(iterVio->second[0], iterVio->second[1], iterVio->second[2]);
                    wTj.block<3, 3>(0, 0) = Quaternionf(iterVioNext->second[3],
                                                        iterVioNext->second[4],
                                                                        iterVioNext->second[5],
                                                                        iterVioNext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Vector3f(iterVioNext->second[0], iterVioNext->second[1], iterVioNext->second[2]);
                    Matrix4f iTj = wTi.inverse() * wTj;
                    Quaternionf iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Vector3f iPj = iTj.block<3, 1>(0, 3);
                    ceres::CostFunction *vioFunc = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                               iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                               0.1, 0.01);
                    problem.AddResidualBlock(vioFunc, nullptr, qArray[i], tArray[i], qArray[i+1], tArray[i+1]);
                }

                iterPrior = priorPoseMap.find(iterVio->first);
                if (iterPrior != priorPoseMap.end()) {
                    ceres::CostFunction* priorTFunc = TError::Create(iterPrior->second[0], iterPrior->second[1], iterPrior->second[2], iterPrior->second[7]);
                    problem.AddResidualBlock(priorTFunc, lossFunction, tArray[i]);
                    if (!priorinit) {
                        priorinit = true;
                        iterPriorinit = iterPrior;
                        initind = i;
                        continue;
                    }
                    else {
                        Matrix3f wTi = Matrix3f::Identity(), wTj = Matrix3f::Identity();
                        wTi.block<3, 3>(0, 0) = Quaternionf(iterPriorinit->second[3], iterPriorinit->second[4], iterPriorinit->second[5], iterPriorinit->second[6]).toRotationMatrix();
                        wTj.block<3, 3>(0, 0) = Quaternionf(iterPrior->second[3], iterPrior->second[4], iterPrior->second[5], iterPrior->second[6]).toRotationMatrix();
                        Matrix3f iTj = wTi.inverse() * wTj;
                        Quaternionf iQj;
                        iQj = iTj.block<3, 3>(0, 0);

                        ceres::CostFunction* priorRFunc = RelativeRError::Create(iQj.w(), iQj.x(), iQj.y(), iQj.z(), 1.0);
                        problem.AddResidualBlock(priorRFunc, nullptr, qArray[initind], qArray[i]);
                    }
                }
            }

            ceres::Solve(options, &problem, &summary);

            iter = globalPoseMap.begin();

            for (auto j = 0; j < length; ++j, ++iter) {
                vector<float> globalPose{ static_cast<float>(tArray[j][0]), static_cast<float>(tArray[j][1]), static_cast<float>(tArray[j][2]), static_cast<float>(qArray[j][0]), static_cast<float>(qArray[j][1]), static_cast<float>(qArray[j][2]), static_cast<float>(qArray[j][3]) };
                iter->second = globalPose;
                if (j == length - 1) {
                    double tt = iter->first;
                    Matrix4f vioTbody = Matrix4f::Identity();
                    Matrix4f priorTbody = Matrix4f::Identity();
                    vioTbody.block<3, 3>(0, 0) = Quaternionf(vioPoseMap[tt][3], vioPoseMap[tt][4], vioPoseMap[tt][5], vioPoseMap[tt][6]).toRotationMatrix();
                    vioTbody.block<3, 1>(0, 3) = Vector3f(vioPoseMap[tt][0], vioPoseMap[tt][1], vioPoseMap[tt][2]);
                    priorTbody.block<3, 3>(0, 0) = Quaternionf(globalPose[3], globalPose[4], globalPose[5], globalPose[6]).toRotationMatrix();
                    priorTbody.block<3, 1>(0, 3) = Vector3f(globalPose[0], globalPose[1], globalPose[2]);
                    PRIOR_T_VIO = priorTbody * vioTbody.inverse();
                }
            }
            updateGlobalPath();
            mVioPoseMap.unlock();
        }
        std::chrono::milliseconds dura(1500);
        std::this_thread::sleep_for(dura);
    }
}

void priorOpt::updateGlobalPath() {
    globalPath.poses.clear();
    vector<float> f = globalPoseMap.begin()->second;
    for (const auto& it : globalPoseMap) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(it.first);
        pose.header.frame_id = "world";
        pose.pose.position.x = it.second[0] - f[0];
        pose.pose.position.y = it.second[1] - f[1];
        pose.pose.position.z = it.second[2] - f[2];
        pose.pose.orientation.w = it.second[3];
        pose.pose.orientation.x = it.second[4];
        pose.pose.orientation.y = it.second[5];
        pose.pose.orientation.z = it.second[6];
        globalPath.poses.emplace_back(pose);
    }
    globalPathPub.publish(globalPath);
    std::ofstream file0(resultDir + "/" + std::to_string(run_cnt) + "/prior_global.txt", std::ios::out);
    for (const auto& it : globalPath.poses) {
        file0 << std::setprecision(16) << it.header.stamp.toNSec() << " " << std::setprecision(12) << it.pose.position.x << " " << it.pose.position.y << " " << it.pose.position.z << " "
              << it.pose.orientation.x << " " << it.pose.orientation.y << " " << it.pose.orientation.z << " " << it.pose.orientation.w << "\n";
    }
    file0.close();
    std::ofstream file(resultDir + "/" + std::to_string(run_cnt) + "/prior.txt", std::ios::out);
    for (const auto& it : globalPath.poses) {
        file << std::setprecision(16) << it.header.stamp.toSec() << " " << std::setprecision(12) << it.pose.position.x << " " << it.pose.position.y << " " << it.pose.position.z << " "
             << it.pose.orientation.x << " " << it.pose.orientation.y << " " << it.pose.orientation.z << " " << it.pose.orientation.w << "\n";
    }
    file.close();
}