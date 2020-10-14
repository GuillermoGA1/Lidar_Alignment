#include "cloud.h"
#include "filters.h"
#include "registrations.h"
#include <pcl/io/pcd_io.h>

    //Function to store the point cloud from a pcd file into a point cloud variable.
    void cloud::read_file(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud) {
        pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *source_cloud);
    }

//Stores the desired parameters for all the filters.
void cloud::set_filter(float x_min,float x_max,float y_min, float y_max, float z_min, float z_max, float leaf, double deviation) {
    x_min_ = x_min;    x_max_ = x_max;
    y_min_ = y_min;   y_max_ = y_max;
    z_min_ = z_min;   z_max_ = z_max;
    leaf_ = leaf;    deviation_ = deviation;
}

//An instance of the filter class is created and the selected filter is applied.
void cloud::apply_filter(int filter_type, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) {
    filters filter;
    if (filter_type == 1) {  //Pass through
        filter.setPassThrough(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
        filter.PassThrough(source_cloud, filtered);
    } else if (filter_type == 2) { //Voxel grid
        filter.setVoxel(leaf_);
        filter.Voxel(source_cloud, filtered);
    } else if (filter_type == 3) { //Outlier removal
        filter.setOutlierRemoval(deviation_);
        filter.OutlierRemoval(source_cloud, filtered);
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
pcl::PointCloud <pcl::PointXYZ>::Ptr cloud::compare_clouds(int method,pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                                           pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud, double icp_distance, double epsilon){
    if (method == 1){ //Simple point to plane ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(reference_cloud);
        // Set the max correspondence distance
        icp.setMaxCorrespondenceDistance(icp_distance);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations(50);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon(epsilon);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon(1.0);

        //pcl::PointCloud<pcl::PointXYZ>cloud_icp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*cloud_icp);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                  icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        return cloud_icp;
    }else if (method == 2){ //Normal distribution transform registration
        registrations ndr;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp = ndr.pointCloudCb(input_cloud,reference_cloud, icp_distance, epsilon);
        return cloud_icp;
    }else if (method == 3){ //Incremental ICP
        registrations inc_icp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp = inc_icp.pointCloudCb_icp_inc(input_cloud,reference_cloud, icp_distance, epsilon);
        transformation_ = inc_icp.get_transform();
        score_ = inc_icp.get_score();
        return cloud_icp;
    }
}


//Function to perform segmentation based on RANSAC
pcl::PointCloud <pcl::PointXYZ>::Ptr cloud::segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double threshold){
    std::vector<int> inliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (input_cloud));
    // Create a RANSAC object based on spherical model
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


