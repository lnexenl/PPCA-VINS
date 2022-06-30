/*******************************************************
* Copyright (C) 2022
* Nanoscale Integrated Circuits and System Lab, Energy Efficient Computing Group (NICS-EFC)
* Department of Electronic Engineering, Tsinghua University.
* Created by Zihan Lin on 2022/3/31.
*******************************************************/

#include "frames.h"

void ndtFrame::addFrame(const dKeyframe& dkf0) {
    R.push_back(dkf0.vio_R);
    T.push_back(dkf0.vio_T);
    pointcloud.push_back(dkf0.filteredPC);
}

void ndtFrame::buildVioLocalCloud() {
    for (const auto& cloud : pointcloud) {
        localCloud += cloud;
    }
}


Matrix4f ndtFrame::registerToPriorMap(pcl::VoxelGrid<PointXYZ>& vg, pcl::Registration<PointXYZ, PointXYZ>& ndt, const PointCloudXYZ::ConstPtr& priorPtr, Eigen::Vector3d& clip_range, Matrix4f& m) {
    //    pcl::Registration<PointCloudXYZ, PointCloudXYZ>::PointCloudTargetPtr tmpPtr(new pcl::Registration<PointCloudXYZ, PointCloudXYZ>::PointCloudTarget);
    PointCloudXYZ::Ptr tmpcloud(new PointCloudXYZ());
    PointCloudXYZ::Ptr srcPtr(new PointCloudXYZ(localCloud));
    PointCloudXYZ::Ptr tgtPtr(new PointCloudXYZ());

    PointXYZ min_pt, max_pt;

    pcl::IndicesPtr ind(new pcl::Indices());
    pcl::getMinMax3D<PointXYZ>(*srcPtr, min_pt, max_pt);
    pcl::BoxClipper3D<PointXYZ> bc(Affine3f::Identity());
    Vector3f sca(1. / ((max_pt.x - min_pt.x) / 2 + clip_range.x()),
                 1. / ((max_pt.y - min_pt.y) / 2 + clip_range.y()),
                 1. / ((max_pt.z - min_pt.z) / 2 + clip_range.z()));
    Vector3f tr((max_pt.x + min_pt.x) / 2, (max_pt.y + min_pt.y) / 2, (max_pt.z + min_pt.z) / 2);
    Affine3f t(Translation3f(-tr.cwiseProduct(sca)));
    //    std::cout << sca << std::endl << tr << std::endl << -tr.cwiseProduct(sca) << std::endl;
    t.scale(sca);
    //    std::cout << t.matrix() << std::endl;
    bc.setTransformation(t);
    bc.clipPointCloud3D(*priorPtr, *ind);
    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(priorPtr);
    ei.setIndices(ind);
    ei.filter(*tgtPtr);

    //    std::cout << min_pt << max_pt << std::endl;

    //    cbf.setNegative(false);
    //    cbf.setInputCloud(priorPtr);
    //    cbf.setMax(Eigen::Vector4f(max_pt.x + clip_range.x(), max_pt.y + clip_range.y(), max_pt.z + clip_range.z(), 1));
    //    cbf.setMin(Eigen::Vector4f(min_pt.x - clip_range.x(), min_pt.y - clip_range.y(), min_pt.z - clip_range.z(), 1));
    //    cbf.filter(*tgtPtr);

    vg.setInputCloud(srcPtr);
    vg.filter(*srcPtr);
    //    ROS_INFO("%d %d %d", cbf.getInputCloud()->size(), tgtPtr->size(), srcPtr->size());
    //    std::cout << t.matrix() << std::endl;
    ROS_INFO("SRC pointcloud: %zu points, TGT pointcloud: %zu points", srcPtr->points.size(), tgtPtr->points.size());
    ndt.setInputSource(srcPtr);
    ndt.setInputTarget(tgtPtr);
    ndt.align(*tmpcloud);
    return ndt.getFinalTransformation();
}
