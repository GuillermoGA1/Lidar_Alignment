#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>
#include "registrations.h"

pcl::PointCloud <pcl::PointXYZ>::Ptr registrations::icpAlign(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr reference, double icp_distance, double epsilon){

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(point_cloud);
    icp.setInputTarget(reference);
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

}



//Function to align point clouds using Normal Distribution Transform (CURRENTLY NOT USED)
Eigen::Matrix4f registrations::pairAlign(const PointCloudRGB::Ptr cloud_src, const PointCloudRGB::Ptr cloud_tgt)
{

    PointCloudRGB::Ptr src(new PointCloudRGB); //source cloud
    PointCloudRGB::Ptr tgt(new PointCloudRGB); //target cloud
    src = cloud_src;
    tgt = cloud_tgt;

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;

    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (icp_distance_);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (epsilon_);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (60);

    // Setting point cloud to be aligned.
    ndt.setInputSource(src);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(tgt);

    // Calculating required rigid transform to align the input cloud to the target cloud.
    PointCloudRGB::Ptr output_cloud (new PointCloudRGB);
    //ndt.align (*output_cloud, init_guess);
    ndt.align (*output_cloud);

    mean_icp_score_ = ndt.getFitnessScore ();

    std::cout << "[NormalDistributionRegistration:] has converged with score of: " << mean_icp_score_ << std::endl;

    return ndt.getFinalTransformation();
}


//Function to iterate using the NDT registration (CURRENTLY NOT USED)
pcl::PointCloud <pcl::PointXYZ>::Ptr registrations::pointCloudCb(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr reference, double icp_distance, double epsilon)
{
    icp_distance_ = icp_distance;
    epsilon_ = epsilon;
    counter_++;

        pointcloud_current_ = *point_cloud;

        // Align point clouds
        Eigen::Matrix4f pair_transform;
        pair_transform = pairAlign(point_cloud,reference);
        std::cout << "Received point cloud number: "<< counter_ << " with " << pointcloud_current_.points.size() << " points." << std::endl;

        // Update the global transform
        final_transformation_ *= pair_transform;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ>);

        // Transform current pair into the global transform
        pcl::transformPointCloud(*point_cloud, *transformed, final_transformation_);
        return transformed;
}



//Function to perform the incremental ICP
Eigen::Matrix4f registrations::pairAlign_inc(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr reference)
{
    int iter = 30;
    double sum_scores = 0.0;

    PointCloudRGB::Ptr src(new PointCloudRGB);
    PointCloudRGB::Ptr tgt(new PointCloudRGB);
    src = point_cloud; //source cloud
    tgt = reference; //target cloud

    // Compute surface normals and curvature using kdTree
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
    pcl::NormalEstimation<PointT , PointNormal> norm_est;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(50);

    //Get the normal point of the source and target clouds
    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);
    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    // Align
    pcl::IterativeClosestPoint<PointNormal, PointNormal> reg;
    reg.setTransformationEpsilon(epsilon_);

    // Set the maximum distance between two correspondences (src<->tgt)
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance(icp_distance_);

    // Set the point representation
    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f ti = Eigen::Matrix4f::Identity(), prev;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(100);

    for (int i = 0; i < iter; ++i)
    {
        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        // Accumulate transformation between each Iteration
        ti = reg.getFinalTransformation() * ti;

        // If the difference between this transformation and the previous one
        // is smaller than the threshold, refine the process by reducing
        // the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
        {
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance() - 0.001);
            std::cout<<"[PairwiseIncrementalRegistration:] Reducing the maximal correspondence distance to " << reg.getMaxCorrespondenceDistance()<< std::endl;
        }

        prev = reg.getLastIncrementalTransformation();

        //UNCOMMENT NEXT LINES TO SEE RESULTS OF EACH ITERATION
        /*if (reg.hasConverged())
        {
            std::cout<<"[PairwiseIncrementalRegistration:] Iteration Nr. " << i << " has converged with score of: " << reg.getFitnessScore()<< std::endl;
        }
        else
        {
            std::cout << "[PairwiseIncrementalRegistration:] Iteration Nr. " << i << " has NOT converged!"<< std::endl;
        } */

        sum_scores += reg.getFitnessScore();
    }

    mean_icp_score_ = sum_scores/iter;

    return ti;
}

//FUnction to run the incremental ICP on two clouds
pcl::PointCloud <pcl::PointXYZ>::Ptr registrations::pointCloudCb_icp_inc(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr reference, double icp_distance, double epsilon)
{
    icp_distance_ = icp_distance;
    epsilon_ = epsilon;
    counter_++;

    pointcloud_current_ = *point_cloud;

    // Align point clouds
    Eigen::Matrix4f pair_transform;
    pair_transform = pairAlign_inc(point_cloud,reference);
    std::cout << "Received point cloud number: "<< counter_ << " with " << pointcloud_current_.points.size() << " points." << std::endl;

    // Update the global transform
    final_transformation_ *= pair_transform;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ>);
    // Transform current pair into the global transform
    pcl::transformPointCloud(*point_cloud, *transformed, final_transformation_);
    return transformed;
}

//Getter for final transformation
Eigen::Matrix4f registrations::get_transform(){
    return final_transformation_;
}

//Getter for the score of the alignment
double registrations::get_score(){
    return mean_icp_score_;
}