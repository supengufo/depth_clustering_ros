cmake_minimum_required(VERSION 2.8.3)
project(depth_cluster_ros)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)
catkin_package(

)

###########
## Build ##
###########
find_package(PCL REQUIRED)
set(Eigen_INCLUDE_DIRS "usr/include/eigen3")
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(pose_curve
#   include/pose_curve.cpp
# )

add_library(depth_cluster include/depth_cluster.cpp)
target_link_libraries(depth_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(depth_cluster ${catkin_EXPORTED_TARGETS} )


add_executable(depth_cluster_node src/main.cpp)
target_link_libraries(depth_cluster_node depth_cluster)

