#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include "cloud.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


std::ofstream myfile; //object to store results of error analysis in a csv file
double final_score; //global variable to keep track of the alignment score

// Define initial value of ROS parameters

//Parameters for translating the reference cloud to the origin of the reference frame
float x_t = 0;
float y_t = 0;
float z_t = 0;
//Parameters for the pass through filter
float x_min_ref = -20;
float x_max_ref = 20;
float y_min_ref = -1; // With this value only leading half of the blade is considered; use  -4 for full blade
float y_max_ref = 0.7; //With this value only leading half of the blade is considered; use  10 for full blade
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
//Set the file from which the reference is taken.
std::string file_ref = "Mesh1.pcd";

//Sensor messages variables to publish point clouds in RVIZ
sensor_msgs::PointCloud2 original_msg;
sensor_msgs::PointCloud2 reference_centered_msg;
sensor_msgs::PointCloud2 filtered_msg;
sensor_msgs::PointCloud2 icp_msg;
sensor_msgs::PointCloud2 icp_temp_msg;

//Function to set ros parameters (change the value of any parameter while running the program.)
void set_ros_parameters(){
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
}

double degreesToRadians(float degrees) {
    return degrees * M_PI / 180;
}

//This function calculates the Euclidean distance between two points given their GPS coordinates.
double distanceBetweenEarthCoordinates(double lat1, double lon1,double lat2,double lon2) {

    double earthRadius_m = 6371000;

    double dLat = degreesToRadians(lat2-lat1);
    double dLon = degreesToRadians(lon2-lon1);

    lat1 = degreesToRadians(lat1);
    lat2 = degreesToRadians(lat2);

    //Haversine function
    double a = sin(dLat/2) * sin(dLat/2) +sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a),sqrt(1-a));

    return earthRadius_m * c;
}


//This callback contains the core algorithm and it runs every time a new point cloud is available from the LiDAR.
void callback(const sensor_msgs::PointCloud2::ConstPtr& input, const sensor_msgs::NavSatFix::ConstPtr& coordinates){

    //Create structures to store filtering parameters for both point clouds
    params reference_cloud_parameters{x_min_ref,x_max_ref,y_min_ref,y_max_ref,z_min_ref,z_max_ref,leaf_ref,deviation_ref};
    params measured_cloud_parameters{x_min,x_max,y_min,y_max,z_min,z_max,leaf,deviation};

    //Retrieve real-time coordinates from subscriber
    double current_lat = coordinates->latitude;
    double current_lon = coordinates->longitude;

    //Fixed coordinates for the reference at the tip of the blade
    double lat1_a = 52.6824388333;
    double lon1_a = 5.92323816667;
    double lat1_b = 52.6824385;
    double lon1_b =  5.92323833333;

    //Fixed coordinate for the reference at the root of the blade
    double lat2_a = 52.6822855;
    double lon2_a = 5.92307783333;
    double lat2_b = 52.6822861667;
    double lon2_b = 5.92307883333;

    //Get the origin of applanix as the middle point in the root and the tip
    double tip_lat = (lat1_a + lat1_b)/2;
    double tip_lon = (lon1_a + lon1_b)/2;
    double root_lat = (lat2_a + lat2_b)/2;
    double root_lon = (lon2_a + lon2_b)/2;

    //Use this to measure distance to longitudinal centre of the blade
    //double blade_lat = (root_lat + tip_lat)/2;
    //double blade_lon = (root_lon + tip_lon)/2;

    //Use this to measure distance to root of the blade
    double blade_lat = root_lat ;
    double blade_lon = root_lon;

    //Calculate the distance between GPS coordinates
    double distance_gps = distanceBetweenEarthCoordinates(blade_lat,blade_lon,current_lat,current_lon);

    //Set variables to store the pointclouds generated through the algorithm

    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_raw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_centered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    // Instantiate two cloud objects to compute the different modules of the algorithm
    cloud reference_pointcloud;
    cloud measured_pointcloud;

    //Define reference and measured clouds
    pcl::fromROSMsg(*input, *source_cloud); //Point cloud from Lidar
    measured_pointcloud.set_source(source_cloud);
    reference_pointcloud.read_file(file_ref);   //Point cloud from CAD file

    //Scale and transform reference to match measurement size
    reference_pointcloud.scale_transform();
    reference_raw = reference_pointcloud.get_cloud(); //Retrieve reference after scaling

    //Get coordinates of the root of the reference
    pcl::PointXYZ minPt;
    pcl::PointXYZ maxPt;
    pcl::getMinMax3D(*reference_raw, minPt, maxPt);
    x_t = maxPt.x; y_t = (minPt.y+maxPt.y)/2;

    //Filter blade from  reference cloud
    reference_pointcloud.set_filter(reference_cloud_parameters);
    reference_pointcloud.apply_filter(Pass_through);  //Passthrough filter
    reference_pointcloud.apply_filter(Voxel);    //Voxel filter

    //Translate the filtered reference point cloud so that the root is located at the origin of the reference frame.
    reference_cloud = reference_pointcloud.get_cloud();
    Eigen::Affine3f center_origin = Eigen::Affine3f::Identity();
    center_origin.translation() << -x_t, -y_t, -z_t;
    pcl::transformPointCloud (*reference_cloud, *reference_centered, center_origin);

    //Filter and segment blade from Lidar measurement
    measured_pointcloud.set_filter(measured_cloud_parameters);
    measured_pointcloud.apply_filter(Pass_through); //Passthrough filter
    measured_pointcloud.apply_filter(Voxel); //Voxel filter
    measured_pointcloud.apply_filter(Outlier); //Outlier removal filter

    //Use the segmentation module to extract the shape of the blade from the whole point cloud
    //pcl::PointCloud<pcl::PointXYZ>::Ptr blade = measured_pointcloud.segment_cloud(seg_threshold);
      pcl::PointCloud<pcl::PointXYZ>::Ptr blade = measured_pointcloud.get_cloud();

    //Execute registration using incremental icp
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp = measured_pointcloud.compare_clouds(incremental_icp, blade,reference_centered, icp_distance, epsilon);

    // Save final transformation matrix and the icp score of the alignment
    Eigen::Matrix4f final_transform = measured_pointcloud.get_transform();
    final_score = measured_pointcloud.get_score();

    //Calculate the norm (Euclidean distance) of the translation applied to the measured point cloud.
    double norm = sqrt(pow(final_transform(0,3),2)+pow(final_transform(1,3),2)+pow(final_transform(2,3),2));

    //Print results
    std::cout << final_transform << std::endl;
    std::cout << "Score: " << final_score << std::endl;
    std::cout<<  "Distance align: " << norm << std::endl;
    std::cout <<  "Distance gps: " << distance_gps << std::endl;

    //Save the alignment procedure to the csv file, only if the alignment was accurate enough.
    if (final_score <= 1) {
        myfile << std::fixed << std::setprecision(11) << current_lat << ',' << current_lon << ','
               << norm << ',' << distance_gps << ',' << abs(norm - distance_gps) << ',' << final_score << "\n";
    }

    //Convert point clouds to ROS messages
    pcl::toROSMsg(*reference_cloud, original_msg);
    original_msg.header.frame_id = "/os1_lidar";

    pcl::toROSMsg(*reference_centered, reference_centered_msg);
    reference_centered_msg.header.frame_id = "/os1_lidar";

    pcl::toROSMsg(*blade, filtered_msg);
    filtered_msg.header.frame_id = "/os1_lidar";

    pcl::toROSMsg(*cloud_icp, icp_msg );
    icp_msg.header.frame_id = "/os1_lidar";

}


int main(int argc, char **argv)
{
    //Initialize ROS node
    ros::init (argc, argv, "align");
    ros::NodeHandle nh;
    myfile.open ("example2.csv");
    set_ros_parameters(); //Set initial parameters

    //Subscribers: one for the point cloud of the lidar and the other for the GPS coordinates.
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub1 (nh, "/os1_cloud_node/points", 500);
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub2(nh, "/applanix_navsatfix", 500);

    //Syncrhonize subscribers
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2 ,sensor_msgs::NavSatFix> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(500), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("reference", 100); //Filtered reference
    ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("reference_centered", 100); //Reference with root at the origin
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("points_filter", 100); //Blade extracted from measurement
    ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("icp", 100); //Resulting alignment

    ros::spinOnce();

    //Run continuously
    while (ros::ok())
    {
        get_ros_parameters();
        ros::Rate loop_rate(10);
        pub.publish(original_msg);
        pub1.publish(reference_centered_msg);
        pub2.publish(filtered_msg);
        if (final_score <= 1) { //Only shows the successful alignments
            pub3.publish(icp_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    myfile.close();

}


