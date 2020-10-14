//
// Created by guillermo on 17-09-20.
//

#ifndef NEW_NODE_FILTERS_H
#define NEW_NODE_FILTERS_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


class filters {
private:
    //Parameters for pass through filter
    float x_min_;    float x_max_;
    float y_min_;   float y_max_;
    float z_min_;   float z_max_;

    float leaf_;     //Parameter for voxel grid filter
    double deviation_; //Parameter for outlier removal filter

public:
    void setPassThrough(float x_min,float x_max,float y_min, float y_max, float z_min, float z_max);
    void setVoxel(float leaf);
    void setOutlierRemoval(double deviation);

    void PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);

    void Voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);

    void OutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);
};



#endif //NEW_NODE_FILTERS_H
