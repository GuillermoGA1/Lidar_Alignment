ALIGNMENT ALGORITHM 

The repository comprises the following files:

A-LOAM pcd files: point clouds obtained from the reconstruction of the blade through A-LOAM. Currently they are not part of the implementation, but can be used to set the reference point cloud. 

Mesh pcd files: point clouds obtained from the conversion of the CAD model into a STL file. Currently used to set the reference point cloud in the algorithm. "Mesh1" is the one set by default, but can be changed to "MeshHQ" (high quality) which contains more points  but makes the algorithm slower. 

Blade pcd files: point clouds obtained directly from the Lidar at different location of the blade. 

SRC folder: contains all the C++ files used for the implementation. Each of the next files are contained in this folder.

aligncad.cpp : this file allows to perform the alignment of the blade.pcd files against the reference (Mesh.pcd). Therefore this files are required in order for the program to work. Most of the functions used come from the cloud class (cloud.cpp). In order to visualize the point clouds in RVIZ a frame named "base_link" needs to be defined while running ROS. To do so, the next line shall be entered in an Ubunut prompt (also written at the end of the code):  
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 50

align_gps_rt : this file allow to perform the alignment of real-time measurements form Lidar agains the reference. Hence, the rosbag containing the real-time measuremente shall be run so the algorithm starts publishing the alignment.This program also uses the cloud class to run most of the functions. Apart from the alignment, this program can also compute the error of the alignment comparing it to the distance computed from GPS coordinates. A CSV file is generated after the program is run containing all the data to do an error analysis.

cloud.cpp: This file contains the functions used by the cloud class. Three kinds of functions can be distinguished: from filtering the point cloud, for segmentation and for registration (alignment). At the same time, thi class calls functions from two sub-classes : filters.cpp and registrations.cpp 

filters.cpp This sub class defines the code for running three different kinds of filters:
1) Pass through filter
2) Voxel grid  filter
3) Outlier removal filter

registrations.cpp : This sub class contains three kinds of registration algorithm which can be used 

