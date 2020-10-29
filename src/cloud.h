//
// Created by guillermo on 07-09-20.

#ifndef NEW_NODE_CLOUD_H
#define NEW_NODE_CLOUD_H
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
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
enum filter_type{Pass_through=0,Voxel=1,Outlier=2};
enum registration_type{icp=0, ndt=1, incremental_icp=2};

//Structure to store parameters for the three kinds of filters
struct params{float x_min;float x_max;float y_min;float y_max; float z_min; float z_max; float leaf; double deviation;};


class cloud{

private:
    //Parameters for pass through filter
       float x_min_;    float x_max_;
       float y_min_;   float y_max_;
       float z_min_;   float z_max_;
       float leaf_;  //Leaf (parameter) to define size of boxes in voxel filter
       double deviation_; //Standrad deviation (parameter) for outlier removal filter

       Eigen::Matrix4f transformation_;
       double score_;
       pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;

public:

    void read_file(std::string file_name);

    void set_filter(params parameters);

    void apply_filter(filter_type selected_filter);

    void scale_transform();

    void set_source(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);

    Eigen::Matrix4f get_transform();
    double get_score();
    pcl::PointCloud <pcl::PointXYZ>::Ptr get_cloud();

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr reg_grow_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    pcl::PointCloud <pcl::PointXYZ>::Ptr segment_cloud(double threshold);
    pcl::PointCloud <pcl::PointXYZ>::Ptr compare_clouds(registration_type method, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                        pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud, double icp_distance, double epsilon);

};



#endif //NEW_NODE_CLOUD_H
