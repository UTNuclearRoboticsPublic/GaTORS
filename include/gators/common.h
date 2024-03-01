#pragma once

// ROS Libraries
// #include <ros/ros.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float64.h>
#include "tf2_ros/transform_listener.h"
// #include <rosbag/bag.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/empty.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker_array.h>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/features/normal_3d_omp.h>
// #include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/region_growing.h>
