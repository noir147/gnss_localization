cmake_minimum_required(VERSION 2.8.3)
project(gnss_localization)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	  geometry_msgs
		sensor_msgs
		roscpp
		std_msgs
		tf
		tf2
		tf2_ros
		)

catkin_package(
	CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs roscpp std_msgs tf tf2 tf2_ros
)

###########
## Build ##
###########

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable( gnss_localization_node
	              src/gnss_localization_node.cpp)

target_link_libraries(gnss_localization_node
	${catkin_LIBRARIES}
	)
