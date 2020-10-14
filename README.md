ALIGNMENT ALGORITHM 

The repository comprises the following files:

A-LOAM pcd files: point clouds obtained from the reconstruction of the blade through A-LOAM. Currently they are not part of the implementation, but can be used to set the reference point cloud. 

Mesh pcd files: point clouds obtained from the conversion of the CAD model into a STL file. Currently used to set the reference point cloud in the algorithm. "Mesh1" is the one set by default, but can be changed to "MeshHQ" (high quality) which contains more points  but makes the algorithm slower. 

Blade pcd files: point clouds obtained directly from the Lidar at different location of the blade. 

SRC folder: contains all the C++ files used for the implementation. Each of the next files are contained in this folder.

aligncad.cpp : this file allows to perform the alignment of the blade.pcd files against the reference (Mesh.pcd). Therefore this files are required in order for the program to work. Most of the functions used come from the cloud class (cloud.cpp). The agorithm uses a two step alignment (coarse with ICP and fine with incremental ICP) In order to visualize the point clouds in RVIZ a frame named "base_link" needs to be defined while running ROS. To do so, the next line shall be entered in an Ubunut prompt (also written at the end of the code):  
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 50

align_gps_rt : this file allow to perform the alignment of real-time measurements form Lidar agains the reference. Hence, a  ROSBAG CONTAINING THE REAL TIME MEASUREMENTS SHALL BE RUN so the algorithm starts publishing the alignment .This program also uses the cloud class to run most of the functions.In this case, alignment is performed in a single step using incremental ICP. Apart from the alignment, this program can also compute the error of the alignment comparing it to the distance computed from GPS coordinates. A CSV file is generated after the program is run containing all the data to do an error analysis.

cloud.cpp: This file contains the functions used by the cloud class. Three kinds of functions can be distinguished: from filtering the point cloud, for segmentation and for registration (alignment). At the same time, thi class calls functions from two sub-classes : filters.cpp and registrations.cpp 

filters.cpp This sub class defines the code for running three different kinds of filters:
1) Pass through filter
2) Voxel grid  filter
3) Outlier removal filter

registrations.cpp : This sub class contains three kinds of registration algorithm which can be used:
1) ICP: used for coarse alignment in aligncad.cpp
2) NDT: currently no used
3) Incremental ICP: used for both aligncad.cpp and align_gps_rt.cpp

ABOUT THE PARAMETERS:

From the parameters defined, only some of them are worth to try with different values while running in ROS:

file_ref: select which point cloud is used as reference (can be changed to aloam or Mesh)

file: only in aligncad.cpp, selects the pointcloud which is going to be aligned with the reference.

x_t,y_t,z_t: in aligncad.cpp they translate the measured blade to a different location. However, in align_gps_rt DO NOT change them, as they are used instead to move the reference blade to the origin of the reference frame. 

theta,phi,psi : in aligncad.cpp, rotate the measured blade to different orientations to try several alignment scenarios. (not used in align_gps_rt).

y_min_ref, y_max_ref: change whether the whole blade is used as reference, or only the half corresponding to the leading edge. The latter is set by default. 

leaf, leaf_ref: modify the amount of points contained in the blade and the reference. The smaller the value, more points are used to define the shape, but the algorithm becomes slower. 

icp_distance: change the maximum distance to which a correspondence is established by ICP. The algorithm is relaxed by incrasing this value, but accuracy diminishes. 



 




