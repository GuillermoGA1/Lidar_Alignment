#include "filters.h"

//Setter for voxel filter
void filters::setVoxel(float leaf) {
    leaf_ = leaf;
}

//Setter for pass through filter
void filters::setPassThrough(float x_min,float x_max,float y_min, float y_max, float z_min, float z_max) {
    x_min_ = x_min;    x_max_ = x_max;
    y_min_ = y_min;   y_max_ = y_max;
    z_min_ = z_min;   z_max_ = z_max;
}

//Setter for outlier removal filter
void filters::setOutlierRemoval(double deviation) {
    deviation_ = deviation;
}

//Function to apply Voxel filter
void filters::Voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vox;
    vox.setLeafSize(leaf_,leaf_,leaf_); //Define size of boxes to create voxel grid
    vox.setInputCloud (source_cloud);
    vox.filter (*filtered);
}

//Function to apply outlier removal filter
void filters::OutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (source_cloud);
    sor.setMeanK (50); //Neighbor points consider for each point
    sor.setStddevMulThresh(deviation_); //Define the standard deviation at which a point becomes an outlier.
    sor.filter (*filtered);
}

//Function to apply pass through filter
void filters::PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    //Filter points outside the boundaries in the x-axis.
    pass.setInputCloud(source_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min_, x_max_);
    pass.filter(*temp_cloud);
    //Filter points outside the boundaries in the y-axis.
    pass.setInputCloud(temp_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min_, y_max_);
    pass.filter(*temp_cloud);
    //Filter points outside the  boundaries in the z-axis.
    pass.setInputCloud(temp_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*filtered);

}


