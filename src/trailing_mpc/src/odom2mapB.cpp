#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
 
double offset_x, offset_y, offset_yaw, offset_roll, offset_pitch;
tf::Transform transform;//2.声明一个变量用来存储转换信息
bool offset = false;
std::string odom_name;

void OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_loc_msg){
    if(offset) return;
    offset_x = init_loc_msg->pose.pose.position.x;
    offset_y = init_loc_msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(init_loc_msg->pose.pose.orientation, quat);
    // tf::Matrix3x3(quat).getRPY(offset_roll, offset_pitch, offset_yaw);//进行转换
    transform.setOrigin(tf::Vector3(offset_x, offset_y, 0));//3. 设置坐标原点，（0.1，0，0.2）为子坐标系激光坐标系base_laser在父坐标系小车base_link坐标系中的坐标，
    // tf::Quaternion q = tf::createQuaternionFromRPY(offset_roll, offset_pitch, offset_yaw);// 4.定义旋转
    transform.setRotation(quat);
    // double tmproll, tmppitch, tmpyaw;
    // tf::Matrix3x3(quat).getRPY(tmproll, tmppitch, tmpyaw);
    // std::cout << "origin : " << offset_x << ", " << offset_y << std::endl;
    // std::cout << "orientation roll: " << tmproll << ", pitch : " <<  tmppitch << ", yaw : " << tmpyaw <<std::endl;
    offset = true;
}

// void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
//     if(!offset) return;
//     transform.getOrigin().setValue(odom_msg->pose.pose.position.x + transform.getOrigin().getX(),odom_msg->pose.pose.position.y + transform.getOrigin().getY(),odom_msg->pose.pose.position.z + transform.getOrigin().getZ());
//     tf::Quaternion dq (odom_msg->pose.pose.orientation.x,odom_msg->pose.pose.orientation.y,odom_msg->pose.pose.orientation.z,odom_msg->pose.pose.orientation.w);
//     tf::Quaternion q0 (transform.getRotation());
//     tf::Quaternion q1 = q0*dq;
//     transform.getRotation().setW(q1.getW());
//     transform.getRotation().setX(q1.getX());
//     transform.getRotation().setY(q1.getY());
//     transform.getRotation().setZ(q1.getZ());
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_odom2map_broadcast");
    ros::NodeHandle n;

    n.param<std::string>("odom2mapB/odom_name", odom_name, " ");

    ros::Subscriber offset_sub = n.subscribe("initialpose", 2, OffsetCallback);
    // ros::Subscriber odom_sub = n.subscribe("/odom", 2, OdomCallback);
    tf::TransformBroadcaster broadcaster;//1.定义一个广播broadcaster

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        if(!offset) continue;
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", odom_name));  //将变换广播出去
        loop_rate.sleep();
    }
 
    return 0;
}