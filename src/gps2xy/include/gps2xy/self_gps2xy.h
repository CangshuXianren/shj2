///<param name="l">经度</param>
///<param name="B">纬度</param>
///<param name="xc">X坐标</param>
///<param name="yc">Y坐标</param>

#ifndef SELFGPS2XY_H
#define SELFGPS2XY_H

#include <math.h>
#include "ros/ros.h"
#include "custom_messages/vehicle_status.h"
// #include "custom_messages/msgNav982Pkt20.h"
#include <sensor_msgs/NavSatFix.h>
// #include "novatel_oem7_msgs/INSPVAX.h"
#include "custom_messages/vehicle_status.h"
#include "custom_messages/planning_info.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#endif
