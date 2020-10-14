#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include "cloud.h"
#include <pcl/visualization/pcl_visualizer.h>




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//Define initial values of ROS parameters
//Parameters for translating and rotating the measured cloud to a different position
float x_t = 0;
float y_t = -4;
float z_t = 3;
float theta = 0; //Rotation on the transversal axis of the blade (pitch)
float phi = 0;  //Rotation on the longitudinal axis of the blade (roll)
float psi = 0; //Rotation on the z-axis of the blade (yaw)
//Parameters for the pass through filter
float x_min_ref = -20;
float x_max_ref = 20;
float y_min_ref = -1;// With this value only leading half of the blade is considered; use  -4 for full blade
float y_max_ref = 0.7; // Wit this value only leading half of the blade is considered; use 10 for full blade
float z_min_ref = -4;
float z_max_ref = 4;
float x_min = -18;
float x_max = 20;
float y_min = -4;
float y_max = 10;
float z_min = -0.7;
float z_max = 4;
//Parameters for voxel filter
float leaf = 0.15;
float leaf_ref = 0.35;
//Parameters for outlier removal ilter
double deviation = 1;
double deviation_ref = 1;
//Parameter for ransac segmentation
double seg_threshold = 0.87;
//Parameters for registration
double icp_distance = 10;
int max_iterations = 40;
double inlier_threshold = 0.1;
double epsilon = 1e-10;
double e_distance = 1;
//Set the file from which the reference and measurement point clouds  are taken.
std::string file_ref = "Mesh1.pcd";
std::string file = "blade2.pcd";

//Function to set ros parameters (change the value of any parameter while running the program)
void set_ros_parameters() {
    ros::param::set("inlier_threshold", inlier_threshold);
    ros::param::set("seg_threshold", seg_threshold);
    ros::param::set("icp_distance", icp_distance);
    ros::param::set("file_ref", file_ref);
    ros::param::set("max_iterations", max_iterations);
    ros::param::set("epsilon", epsilon);
    ros::param::set("e_distance", e_distance);
    ros::param::set("leaf", leaf);
    ros::param::set("leaf_ref", leaf_ref);
    ros::param::set("deviation", deviation);
    ros::param::set("deviation_ref", deviation_ref);
    ros::param::set("x_min_ref", x_min_ref);
    ros::param::set("x_max_ref", x_max_ref);
    ros::param::set("y_min_ref", y_min_ref);
    ros::param::set("y_max_ref", y_max_ref);
    ros::param::set("z_min_ref", z_min_ref);
    ros::param::set("z_max_ref", z_max_ref);
    ros::param::set("x_min", x_min);
    ros::param::set("x_max", x_max);
    ros::param::set("y_min", y_min);
    ros::param::set("y_max", y_max);
    ros::param::set("z_min", z_min);
    ros::param::set("z_max", z_max);
    ros::param::set("x_t", x_t);
}

//Function  to update the parameters in every ROS loop.
void get_ros_parameters(){
    ros::param::get("inlier_threshold", inlier_threshold);
    ros::param::get("seg_threshold", seg_threshold);
    ros::param::get("icp_distance", icp_distance);
    ros::param::get("file_ref", file_ref);
    ros::param::get("max_iterations", max_iterations);
    ros::param::get("epsilon", epsilon);
    ros::param::get("e_distance", e_distance);
    ros::param::get("leaf", leaf);
    ros::param::get("leaf_ref", leaf_ref);
    ros::param::get("deviation", deviation);
    ros::param::get("deviation_ref", deviation_ref);
    ros::param::get("x_min_ref",x_min_ref);
    ros::param::get("x_max_ref",x_max_ref);
    ros::param::get("y_min_ref",y_min_ref);
    ros::param::get("y_max_ref",y_max_ref);
    ros::param::get("z_min_ref",z_min_ref);
    ros::param::get("z_max_ref",z_max_ref);
    ros::param::get("x_min",x_min);
    ros::param::get("x_max",x_max);
    ros::param::get("y_min",y_min);
    ros::param::get("y_max",y_max);
    ros::param::get("z_min",z_min);
    ros::param::get("z_max",z_max);
    ros::param::get("x_t", x_t);
    ros::param::get("y_t", y_t);
    ros::param::get("z_t", z_t);
    ros::param::get("theta", theta);
    ros::param::get("phi", phi);
    ros::param::get("psi", psi);
}

int main(int argc, char **argv)
{
    //Initialize a ROS node
    ros::init (argc, argv, "align");
    ros::NodeHandle nh;
    cloud ref;
    cloud measure;

    set_ros_parameters(); //Set initial values for parameters

    //Set publishers for ROS topics
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("reference", 10);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("points_filter", 10);
    ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("icp", 10);

    //Define variables to store ROS messages
    sensor_msgs::PointCloud2 original_msg;
    sensor_msgs::PointCloud2 filtered_msg;
    sensor_msgs::PointCloud2 icp_msg;


    //Set pointclouds that will be used through the program
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_raw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_pre (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference (new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud::Ptr source_cloud (new PointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pre_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr svd_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    //Run continuously
    while (ros::ok())
    {

        get_ros_parameters(); //Update ROS parameters

        ref.read_file(file_ref,reference_raw); //Store the reference point cloud into a variable

        //Scale and transform reference to match measurement size
        Eigen::Affine3f scale = Eigen::Affine3f::Identity();
        scale(0,0) = scale (0,0) * 0.001;
        scale (1,1) = scale (1,1) * 0.001;
        scale (2,2) = scale (2,2) * 0.001;
        scale.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitY()));
        scale.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*reference_raw, *reference_raw, scale);

        //Filter  blade from  reference cloud
        ref.set_filter(x_min_ref,x_max_ref,y_min_ref,y_max_ref,z_min_ref,z_max_ref,leaf_ref,deviation_ref);
        ref.apply_filter(1,reference_raw,reference_pre); //Pass through filter
        ref.apply_filter(2,reference_pre,reference); //Voxel filter

        //Filter blade from measured cloud
        measure.read_file(file,source_cloud); //Store measured point cloud into a variable
        measure.set_filter(x_min,x_max,y_min,y_max,z_min,z_max,leaf,deviation);
        measure.apply_filter(1,source_cloud, pre_filtered); //Pass through filter
        measure.apply_filter(2,pre_filtered, filtered_cloud); //Voxel filter

        //Perform segmentation to extract the shape of the blade from the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr blade = measure.segment_cloud(filtered_cloud, seg_threshold);

        //Transform measured cloud to a different location
        Eigen::Affine3f transform = Eigen::Affine3f::Identity(); //Define transformation object
        transform.translation() << x_t, y_t, z_t;
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
        transform.rotate (Eigen::AngleAxisf (phi, Eigen::Vector3f::UnitX()));
        transform.rotate (Eigen::AngleAxisf (psi, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud (*blade, *transformed_cloud, transform); //Execute transformation


        //Execute registration in two steps: coarse alignment with ICP and fine alignment with incremental ICP
        std::cout << "First alignment:"<< std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_temp1 = measure.compare_clouds(1,transformed_cloud, reference, icp_distance, epsilon);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp = measure.compare_clouds(3, transformed_cloud,reference, icp_distance, epsilon);
        Eigen::Matrix4f final_transform = measure.get_transform();
        double final_score = measure.get_score();
        std::cout <<"Second alignment:" << '\n' << final_transform << std::endl;
        std::cout << "Score: " << final_score << std::endl;

        ros::Rate loop_rate(2);

        //Convert clouds to ROS messages
        pcl::toROSMsg(*reference, original_msg);
        original_msg.header.frame_id = "/base_link";
        pub.publish (original_msg); //Publish reference cloud (CAD model)

        pcl::toROSMsg(*transformed_cloud, filtered_msg);
        filtered_msg.header.frame_id = "/base_link";
        pub2.publish (filtered_msg); //Publish measured cloud moved to a different location

        pcl::toROSMsg(*cloud_icp, icp_msg );
        icp_msg.header.frame_id = "/base_link";
        pub3.publish (icp_msg);  //Publish alignment of measured cloud to the reference

        ros::spinOnce ();
        loop_rate.sleep ();
    }
}
//rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 50

