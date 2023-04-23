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

#include <proto/fm.pb.h>
#include <custom_messages/status15.h>
#include <custom_messages/chief_cmd.h>
#include "std_msgs/Int32.h"

double nowyaw;

//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
double offset_x, offset_y, offset_yaw, offset_pitch, offset_roll;
double dp_x,dp_y,dp_z,dp_w;
nav_msgs::Odometry offset;
ros::Publisher odom15_pub, mycloud_vis_pub, odom_rviz_pub;
bool offset_1 = false; // manually offset
bool offset_2; // double offset from mapmerge

// reconfiguration
int ID;
int chieffake_flag = 0;

void ChieffakeCallback(const std_msgs::Int32::ConstPtr& Chieffake_msg) {
  if (ID == 1) {
    chieffake_flag = Chieffake_msg->data;
  }
}

void OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& offset_msg){
  if(offset_1 && offset_2) return;
  std::cout << "offset_1 : " << offset_1 << std::endl;
  std::cout << "offset_2 : " << offset_2 << std::endl;
  offset_x = offset_msg->pose.pose.position.x;
  offset_y = offset_msg->pose.pose.position.y;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(offset_msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(offset_roll, offset_pitch, offset_yaw);
  dp_x = offset_msg->pose.pose.orientation.x;
  dp_y = offset_msg->pose.pose.orientation.y;
  dp_z = offset_msg->pose.pose.orientation.z;
  dp_w = offset_msg->pose.pose.orientation.w;
  if (offset_1 == false) {
    offset_1 = true;
    ROS_WARN("offset initial done");
  } else if (offset_1 == true) {
    offset_2 = true;
    ROS_WARN("offset from mapmerge done");
  }
}

// 转换到绝对坐标系
inline void coneTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d test)
{
    po[0] = pi[0] * cos(test[2]) - pi[1] * sin(test[2]) + test[0];
    po[1] = pi[0] * sin(test[2]) + pi[1] * cos(test[2]) + test[1];
}

//转换到相对坐标系
inline void coneAntiTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d& test)
{
    po[0] = pi[0] * cos(test[2]) + pi[1] * sin(test[2]) - (test[0] * cos(test[2]) + test[1] * sin(test[2]));
    po[1] = -pi[0] * sin(test[2]) + pi[1] * cos(test[2]) - (-test[0] * sin(test[2]) + test[1] * cos(test[2]));   
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
  if(!offset_1) return;
  offset = *odom_msg;
  // std::cout << "before odom: x = " << odom_msg->pose.pose.position.x << ", y = " << odom_msg->pose.pose.position.y << std::endl;
  offset.header.frame_id = "map";
  Eigen::Vector2d pi, po;
  pi << offset.pose.pose.position.x, offset.pose.pose.position.y;
  Eigen::Vector3d move;
  move << offset_x, offset_y, offset_yaw;
  coneTrans(pi, po, move);
  offset.pose.pose.position.x = po[0];
  offset.pose.pose.position.y = po[1];
  // std::cout << "move: offset_x = " << offset_x << ", offset_y = " << offset_y << std::endl;
  // std::cout << "after odom: x = " << offset.pose.pose.position.x << ", y = " << offset.pose.pose.position.y << std::endl;

  // manipulate rpy, and give quaternion finally
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
  double tmproll, tmppitch, tmpyaw;
  tf::Matrix3x3(quat).getRPY(tmproll, tmppitch, tmpyaw);
  nowyaw = offset_yaw + tmpyaw;
  // std::cout<<"nowyaw , offset_yaw , tmpyaw" << nowyaw << " | " << offset_yaw << " | " << tmpyaw << std::endl;
  tf::Quaternion now(0, 0, nowyaw); // roll is around zaxis here actually!!!
  now.normalize();
  offset.pose.pose.orientation.w = now.w();
  offset.pose.pose.orientation.x = now.x();
  offset.pose.pose.orientation.y = now.y();
  offset.pose.pose.orientation.z = now.z();
  
  // reconstruction
  offset.pose.pose.position.z = ID; // use offset.z to claim ID
}

void ScoutbaseCallback(const nav_msgs::Odometry::ConstPtr& scout_base_msg) {
  if(!offset_1) return;
  // if (reconfiguration_flag) {
  //   return;
  // }
  offset.twist.twist.linear.x = scout_base_msg->twist.twist.linear.x;

  custom_messages::status15 offset15;
  offset15.position.x = offset.pose.pose.position.x;
  offset15.position.y = offset.pose.pose.position.y;
  offset15.pose.yaw = nowyaw;
  offset15.linear_velocity.x = offset.twist.twist.linear.x;
  offset15.request = chieffake_flag;

  odom15_pub.publish(offset15);
  odom_rviz_pub.publish(offset);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  Eigen::Quaterniond q (offset.pose.pose.orientation.w, offset.pose.pose.orientation.x, offset.pose.pose.orientation.y, offset.pose.pose.orientation.z);
  Eigen::Vector3d t (offset.pose.pose.position.x, offset.pose.pose.position.y, 0.0);
  Eigen::Matrix4d T (Eigen::Matrix4d::Identity());
  // T.block(0,0,3,3) = q.toRotationMatrix();
  T.block(0,3,3,1) = t;

  pcl::transformPointCloud(*cloud, *cloud, T);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cloud, msg_out);
  msg_out.header.frame_id = "map";
  mycloud_vis_pub.publish(msg_out);
}

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "loc15");  

  ros::NodeHandle n;

  n.param<int>("recv_ID", ID, 0);
  n.param<bool>("self_gps2xy/offset_2", offset_2, true);

  mycloud_vis_pub = n.advertise<sensor_msgs::PointCloud2>("mycloud_vis", 10); 
  odom15_pub = n.advertise<custom_messages::status15>("odom15", 1);
  odom_rviz_pub = n.advertise<nav_msgs::Odometry>("odom15_vis", 1);
  
  ros::Subscriber reconfiguration_sub = n.subscribe("/reID", 1, ChieffakeCallback);
  ros::Subscriber offset_sub = n.subscribe("initialpose", 1, OffsetCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 1, OdomCallback);
  ros::Subscriber speed_sub = n.subscribe("base_odom", 1, ScoutbaseCallback);
  ros::Subscriber cloud_sub = n.subscribe("velodyne_points", 1, cloudCallback);

  ros::Rate rate(50);
  while (ros::ok()) {
    chieffake_flag = 0;
    ros::spinOnce();
    rate.sleep();
  }  

  std::cout << "[loc15] node ID = " << ID << std::endl;  

  return 0;  
}  