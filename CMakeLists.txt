cmake_minimum_required(VERSION 3.10.2)
project(new_node)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	message_filters
	pcl_conversions
	std_msgs
	tf
)

find_package(PCL 1.11 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${pcl_conversions_DIR})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions message_filters)

list(APPEND CLOUD "src/cloud.cpp")
list(APPEND FILTERS "src/filters.cpp")
list(APPEND REGISTRATIONS "src/registrations.cpp")


add_executable(aligncad src/aligncad.cpp ${CLOUD} ${FILTERS} ${REGISTRATIONS})
add_executable(align_gps_rt src/align_gps_rt.cpp ${CLOUD} ${FILTERS} ${REGISTRATIONS})


target_link_libraries(aligncad ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(align_gps_rt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})


