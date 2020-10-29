# LiDAR point cloud alignment algorithm

This project contains a prototype algorithm for performing point cloud registration of measurements obtained from a LiDAR sensor against a reference point cloud obtained from a CAD model of the object of interest. All programs are coded to run through a ROS node.  The algorithm is intended to recognize the blade of a wind turbine as the object of interest. 

# Main features

  - Filtering of a point cloud 
  - Segmentation through RANSAC
  - ICP registration


You can also:
  - Modify filtering and registration parameters in real-time 
  - Change location of the blade to try out different scenarios.
  - Add or delete filtering, segmentation and registration modules to adapt the structure of the algorithm. 

### Requirements

The following software is necessary to run and edit the code:

* Ubuntu 18.04 
* ROS melodic (full version)
* PCL library 1.11.1
* PCL perception package
* A C++ editor (CLion was used  for all coding )

### Installation

First get the full installation of ROS melodic

```sh
$ sudo apt install ros-melodic-desktop-full
```

Then install PCL functions using:
```sh
sudo apt install libpcl-dev
```

Then, to install PCL library 1.11.1, download repository from: https://github.com/PointCloudLibrary/pcl

After donwloading the repository, open a terminal inside the donwloaded folder and enter the following instructions:

```sh
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```
Notice that after the instruction "make", the computer will take some time in compiling all the functions(around 40 min). In case something fails or any dependency is missing, check the next link for mor information: https://pointclouds.org/documentation/tutorials/compiling_pcl_posix.html

Now it is time to create the workspace. First make sure to have the  required catkin tools:
```sh
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```
Then create a folder for the catkin workspace, and a source folcer "src" inside the workspace. Inside the src folder, download the pcl perception package (ros melodic version) available at: https://github.com/ros-perception/perception_pcl

Then, download the folder of this repository and change its name to "new_node". Copy and paste the folder inside the "src" folder from the previously created catkin workspace. 

Open a terminal inside the catkin workspace and run "catkin build". If the installation was succesful the workspace will build the new_node package. 

If the build instruction fails, try the next command
```sh
$ sudo gedit ~/.bashrc
```
This will open a window showing some compiling instructions. Go at the end of the code and check the following line is there:
```sh
source ~/catkin_ws/devel/setup.bash
```
Save the file and open a new terminal. Try building the catkin workspace again. 




### What is in this repository?

 - **A-LOAM pcd files**: point clouds  from the reconstruction of the blade through A-LOAM. Currently they are not part of the implementation, but can be used to set the reference point cloud.
 - **Mesh pcd files**: point clouds obtained from the conversion of the CAD model into a STL file. Currently used to set the reference point cloud in the algorithm. "Mesh1" is the one set by default, but can be changed to "MeshHQ" (high quality) which contains more points but makes the algorithm slower.
 - **Blade pcd files**: measured point clouds obtained directly from the LiDAR at different locations of the blade. They are only used when running "aligncad.cpp".
 - **aligncad.cpp** : this file allows to perform the alignment of the blade.pcd files against the reference (Mesh.pcd). Most of the functions used come from the cloud class (cloud.cpp). The agorithm uses a two step alignment (coarse with ICP and fine with incremental ICP). 

- **align_gps_rt.cpp** : this file allow to perform the alignment of real-time measurements form LiDAR agains the reference. Hence, a ROSBAG CONTAINING THE REAL TIME MEASUREMENTS SHALL BE RUN so the algorithm starts publishing the alignment .This program also uses the cloud class to run most of the functions. In this case, alignment is performed in a single step using incremental ICP. Apart from the alignment, this program can also compute the error of the alignment comparing it to the distance computed from GPS coordinates. A CSV file is generated after the program is run, containing all the data to do an error analysis.
- **cloud.cpp**: This file contains the functions used by the cloud class. Three kinds of functions can be distinguished: for filtering the point cloud, for segmentation and for registration (alignment). At the same time, this class calls functions from two sub-classes : filters.cpp and registrations.cpp
- **filters.cpp** This sub class defines the code for running three different kinds of filters:
    1. Pass through filter
    2. Voxel grid filter
    3. Outlier removal filter
- **registrations.cpp** : This sub class contains three kinds of registration algorithm which can be used:
    1. ICP: used for coarse alignment in aligncad.cpp
    2. NDT: currently no used
    3. Incremental ICP: used for both aligncad.cpp and align_gps_rt.cpp

#### Run the programs 
After finishing all the installation and having built the catkin workspace it is possible to start doing some trials with the programs. The "new_node" package has two nodes: aligncad which takes the measurements from the pcd files contained in this repository, and align_gps_rt which takes measurements from a rosbag. 

**Aligncad**
To run this node, first start a roscore. Then go into the "new_node" folder and open a terminal. Input the next command:
  ```sh
$ rosrun new_node aligncad
```
The terminal will start displaying the transformation matrices continuously. To visualize the alignment input the following line in a different tab to create a reference frame:
  ```sh
$ rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 50
```
Then open rviz and change the frame to "base link". Add the pointclouds from the topics that are published:
    1. reference: publishes the reference cloud obtained from the CAD model
    2. points_filter: measured point cloud corresponding to the blade
    3. icp: alignment of the points_filter cloud towards the reference cloud.
    
**Align_gps_rt**
Because this node works with real-time measurements, first it is necessary to play a rosbag. Start by initializing a roscore. Then go to the folder in which the rosbag is located, open a terminal and input: 
  ```sh
$ rosbag play file.bag
```
Where "file" is the name of the rosbag. Once it is playing start the algorithm with:
  ```sh
$ rosrun new_node align_gps_rt
```
When the program starts running, it will display in the terminal the transformation matrix of the alignment, the score of the alignment and the distance obtained from both the GPS and the transformation matrix. 
Similarly the point clouds can be visualized in RVIZ. In this case the reference frame is already defined from the LiDAR, so in RVIZ choose the "os1_lidar" frame. Same topics as in aligncad are available to publish and one more called "reference centered" which shows the reference point cloud placed at the origin of the reference frame. 

#### How to use ROS parameters?
As mentioned before, one of the functions of this algorithm is that some of the alignment parameters can be changed while the program is runnning. This feature makes it easier to understand the effect of each parameter and test different scenarios. To modify a parameter, input the following line on an Ubuntu terminal once the node is running:

  ```sh
$ rosparam set parameter value
```
Make sure to change "parameter" for the name of the parameter to modify and "value" for the numerical value that should be assigned. To see  a list of the available parameters run the line:
  ```sh
$ rosparam list
```
From all the parameters, the following are the ones which have an important influence on the performance. Thus, it might be interesting to modify them to analyze different scenarios: 
- **file_ref**: select which point cloud is used as reference (can be changed to "aloam.pcd" or "Mesh.pcd").
- **file**: only available in aligncad.cpp, selects the pointcloud which is going to be aligned with the reference (e.g. "blade1.pcd").
- **x_t,y_t,z_t**: in aligncad.cpp they translate the measured blade to a different location in the respective axis. However, in align_gps_rt DO NOT change them, as they are used instead to place the reference blade into the origin of the reference frame.
- **phi,theta,psi** : only available in aligncad.cpp, rotate the measured blade in x,y and z respectively to try different orientations of the measured blade.
- **y_min_ref, y_max_ref**:  define the interval of the pass through filter in the y-axis for the reference cloud. They are used to change whether the whole blade is used as reference, or only the half corresponding to the leading edge. The latter is set by default.
- **leaf, leaf_ref**: modify the voxel size of the voxel grid filter for the measured and the reference cloud respectively. The smaller the value, more points are used to define the point cloud. 
- **icp_distance**: change the maximum distance to which a correspondence is established by ICP. The registration is relaxed by incrasing this value, but accuracy diminishes.
