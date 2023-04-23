#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
 
nav_msgs::Odometry offset;
nav_msgs::Odometry odom;
tf::StampedTransform transform;//2.声明一个变量用来存储转换信息
ros::Publisher myodom_pub;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    offset = *odom_msg;
    offset.child_frame_id = "map";
    offset.pose.pose.position.x = offset.pose.pose.position.x + transform.getOrigin().x();
    offset.pose.pose.position.y = offset.pose.pose.position.y + transform.getOrigin().y();
    // offset.pose.pose.orientation.w = offset.pose.pose.orientation.w + transform.getRotation().getW();
    // offset.pose.pose.orientation.x = offset.pose.pose.orientation.x + transform.getRotation().getX();
    // offset.pose.pose.orientation.y = offset.pose.pose.orientation.y + transform.getRotation().getY();
    // offset.pose.pose.orientation.z = offset.pose.pose.orientation.z + transform.getRotation().getZ();
    tf::Quaternion dq (transform.getRotation());
    tf::Quaternion q0 (offset.pose.pose.orientation.x, offset.pose.pose.orientation.y, offset.pose.pose.orientation.z, offset.pose.pose.orientation.w);
    tf::Quaternion q1 = q0.normalized() *dq.normalized();
    q1.normalize();
    offset.pose.pose.orientation.w = q1.w();
    offset.pose.pose.orientation.x = q1.x();
    offset.pose.pose.orientation.y = q1.y();
    offset.pose.pose.orientation.z = q1.z();
    myodom_pub.publish(offset);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_odom2map_listen");
    ros::NodeHandle n;

    ros::Subscriber odom_sub = n.subscribe("odom", 2, OdomCallback);
    myodom_pub = n.advertise<nav_msgs::Odometry>("myodom", 1);
    tf::TransformListener listener;//1.定义一个广播broadcaster

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        try{
            listener.waitForTransform("odom", "map", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("odom", "map", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
 
    return 0;
}