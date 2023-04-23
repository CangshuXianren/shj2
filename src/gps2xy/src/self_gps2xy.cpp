#include "self_gps2xy.h"

//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
ros::Publisher location_pub;
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
bool init = false;
custom_messages::vehicle_status init_pose; // 用xPos和yPos分别暂存经度和纬度

double offset_x, offset_y, offset_yaw, offset_pitch, offset_roll;
double dp_x,dp_y,dp_z,dp_w;
nav_msgs::Odometry offset;
ros::Publisher myodom_pub, mycloud_vis_pub;
bool offset_1 = false; // manually offset
bool offset_2; // double offset from mapmerge

// reconfiguration
int ID;
bool reconfiguration_flag = false;

void ReconfigurationCallback(const geometry_msgs::Point::ConstPtr& reconstruction_msg){ // x: 1-add, 2-quit, y: ID
    int mode = reconstruction_msg->x;
    int reID = reconstruction_msg->y;
    reconfiguration_flag = true;
    if (mode == 1) {
        if (ID == 4) {
            ID = reID;
        } else if (ID >= reID) {
            ++ID;
        }
    } else if (mode == 2) {
        if (ID == reID) {
            ros::shutdown();
        } else if (ID > reID) {
            --ID;
        }
    } else {
        ROS_WARN("wrong reconfiguration mode");
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
  double nowyaw = offset_yaw + tmpyaw;
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
  if (reconfiguration_flag) {
    return;
  }
  offset.twist.twist.linear.x = scout_base_msg->twist.twist.linear.x;
  offset.twist.twist.angular.z = scout_base_msg->twist.twist.angular.z;

  myodom_pub.publish(offset);
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
 
  ros::init(argc, argv, "self_GPS2XY");  

  ros::NodeHandle n;

  n.param<int>("ID", ID, 0);
  n.param<bool>("self_gps2xy/offset_2", offset_2, true);

  mycloud_vis_pub = n.advertise<sensor_msgs::PointCloud2>("mycloud_vis", 10); 
  myodom_pub = n.advertise<nav_msgs::Odometry>("myodom", 1);
  
  ros::Subscriber reconfiguration_sub = n.subscribe("/reID", 1, ReconfigurationCallback);
  ros::Subscriber offset_sub = n.subscribe("initialpose", 1, OffsetCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 1, OdomCallback);
  ros::Subscriber speed_sub = n.subscribe("base_odom", 1, ScoutbaseCallback);
  ros::Subscriber cloud_sub = n.subscribe("velodyne_points", 1, cloudCallback);

  ros::Rate rate(50);
  while (ros::ok()) {
    reconfiguration_flag = false;
    ros::spinOnce();
    rate.sleep();
  }  

  std::cout << "[localization] node ID = " << ID << std::endl;  

  return 0;  
}  