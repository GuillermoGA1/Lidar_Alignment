#include "cloud.h"
#include "filters.h"
#include "registrations.h"
#include <pcl/io/pcd_io.h>

//Function to store the point cloud from a pcd file into a point cloud variable.
void cloud::read_file(std::string file_name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud);
        source_cloud_ = cloud;
    }

//Function to assign the cloud from an already existing point cloud variable (only for align_gps_rt)
void cloud::set_source(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = source_cloud;
    source_cloud_ = cloud;
}

//Function to scale  the point cloud of the CAD model to the proper dimensions and assign the axes as given by the LiDAR
void cloud::scale_transform(){
        Eigen::Affine3f scale = Eigen::Affine3f::Identity();
        scale(0,0) = scale (0,0) * 0.001;
        scale (1,1) = scale (1,1) * 0.001;
        scale (2,2) = scale (2,2) * 0.001;
        scale.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitY()));
        scale.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*source_cloud_, *source_cloud_, scale);
}

//Stores the desired parameters for all the filters.
void cloud::set_filter(params parameters) {
    x_min_ = parameters.x_min;    x_max_ = parameters.x_max;
    y_min_ = parameters.y_min;   y_max_ = parameters.y_max;
    z_min_ = parameters.z_min;   z_max_ = parameters.z_max;
    leaf_ = parameters.leaf;    deviation_ = parameters.deviation;
}

//An instance of the filter class is created and the selected filter is applied.
void cloud::apply_filter(filter_type selected_filter){
    filters filter;

    switch (selected_filter) {
        case 0: //Pass through
            filter.setPassThrough(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
            filter.PassThrough(source_cloud_, source_cloud_);
            break;
        case 1: //Voxel grid
            filter.setVoxel(leaf_);
            filter.Voxel(source_cloud_, source_cloud_);
            break;
        case 2: //Outlier removal
            filter.setOutlierRemoval(deviation_);
            filter.OutlierRemoval(source_cloud_, source_cloud_);
            break;
        default :
            std::cout << "Invalid filter selection" << std::endl;
            break;
    }
}


//Region growing algorithm which can be used for segmentation (CURRENTLY NOT USED IN THE IMPLEMENTATION)
pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud::reg_grow_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //Get normal points from the point cloud
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(input_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);
    
    //Use a region growing object and set the parameters to segment the different clusters
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (300);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (input_cloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (  7.0/180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    //Separate the point cloud into the different clusters
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored = reg.getColoredCloud ();

    //The biggest clustered is considered to be the blade
    pcl::PointIndicesPtr blade_cloud(new pcl::PointIndices);
    blade_cloud->indices = clusters[0].indices;

    //Extract only the points that correspond to the blade cluster
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_output;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input_cloud);
    extract.setIndices(blade_cloud);
    /*extract.setNegative (false);*/
    extract.filter (*cloud_output);

    return colored;
}

//This function creates an instance of the registrations class and applies the desired registration algorithm
pcl::PointCloud <pcl::PointXYZ>::Ptr cloud::compare_clouds(registration_type method,pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                           pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud, double icp_distance, double epsilon){
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;
    registrations registration;
    switch(method){
        case 0:  //Simple point to plane ICP
            aligned_cloud = registration.icpAlign(input_cloud,reference_cloud, icp_distance, epsilon);
            return aligned_cloud;
            break;
        case 1:   //Normal distribution transform registration
            aligned_cloud = registration.pointCloudCb(input_cloud,reference_cloud, icp_distance, epsilon);
            return aligned_cloud;
            break;
        case 2:  //Incremental ICP
            aligned_cloud = registration.pointCloudCb_icp_inc(input_cloud,reference_cloud, icp_distance, epsilon);
            transformation_ = registration.get_transform();
            score_ = registration.get_score();
            return aligned_cloud;
            break;
        default:
            std::cout << "Invalid registration method" << std::endl;
            break;
    }
}


//Function to perform segmentation based on RANSAC
pcl::PointCloud <pcl::PointXYZ>::Ptr cloud::segment_cloud(double threshold){
    std::vector<int> inliers;  //Vector to store inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    input_cloud = source_cloud_; //Pass the point cloud to a local variable inside the function
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>); //Declare output cloud

    // Create a RANSAC object based on spherical model
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (input_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    //Set parameters for segmentation
    ransac.setDistanceThreshold (threshold);
    ransac.computeModel();
    ransac.getInliers(inliers); //Store the points belonging to the blade as the inliers.
    pcl::copyPointCloud (*input_cloud, inliers, *output_cloud);
    return output_cloud;
}

//Getter for final transformation
Eigen::Matrix4f cloud::get_transform(){
    return transformation_;
}

//Getter for final score
double cloud::get_score(){
    return score_;
}

pcl::PointCloud <pcl::PointXYZ>::Ptr cloud::get_cloud(){
    return source_cloud_;
};



