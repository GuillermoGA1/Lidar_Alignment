//
// Created by guillermo on 22-09-20.
//

#ifndef NEW_NODE_REGISTRATIONS_H
#define NEW_NODE_REGISTRATIONS_H

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>


typedef pcl::PointXYZ                 PointT;
typedef pcl::PointCloud<PointT>       PointCloudRGB;
typedef pcl::PointNormal              PointNormal;
typedef pcl::PointCloud<PointNormal>  PointCloudWithNormals;

class registrations {

public:
    Eigen::Matrix4f pairAlign(const PointCloudRGB::Ptr cloud_src, const PointCloudRGB::Ptr cloud_tgt);

    pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloudCb(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr reference,double icp_distance, double epsilon);

    Eigen::Matrix4f get_transform();

    double get_score();

    Eigen::Matrix4f pairAlign_inc(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr reference);

    pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloudCb_icp_inc(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr reference, double icp_distance, double epsilon);

private:
    int input_cue_;
    int max_number_of_iterations_icp_;
    int counter_;
    double downsample_voxel_size_;
    double epsilon_transformation_;
    double max_correspondence_distance_;
    double euclidean_fitness_epsilon_;
    double mean_icp_score_;
    bool first_cloud_received_ = false;
    double icp_distance_;
    double epsilon_;

    Eigen::Matrix4f final_transformation_ = Eigen::Matrix4f::Identity();
    PointCloudRGB pointcloud_current_;
    PointCloudRGB pointcloud_previous_;
    PointCloudRGB pointcloud_transformed_;
    PointCloudRGB pointcloud_merged_;

};

#endif //NEW_NODE_REGISTRATIONS_H
