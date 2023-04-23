#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

ros::Publisher vir_odom_pub;
ros::Publisher vir_speed_pub;
double v = 0.0;
double w = 0.0;
double yaw = 0.0;
nav_msgs::Odometry vir_odom;
nav_msgs::Odometry vir_speed_tmp;

void CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msgs){
    v = cmd_msgs->linear.x;
    w = cmd_msgs->angular.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_base");
    ros::NodeHandle nh;

    ros::Subscriber mypoint = nh.subscribe("cmd_vel", 2, CmdCallback);

    vir_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    vir_speed_pub = nh.advertise<nav_msgs::Odometry>("base_odom", 1);

    vir_odom.header.frame_id = "map";
    vir_odom.header.stamp = ros::Time::now();
    vir_speed_tmp.child_frame_id = "map";
    vir_speed_tmp.header.stamp = ros::Time::now();

    ros::Rate rate(100);
    double deltaT = 1. / 100.;
    while(ros::ok())
    {
        ros::spinOnce();

        vir_odom.pose.pose.position.x += v * deltaT * cos(yaw);
        vir_odom.pose.pose.position.y += v * deltaT * sin(yaw);
        vir_odom.twist.twist.linear.x = v;
        vir_odom.twist.twist.angular.z = w;
        yaw += w * deltaT;
        tf::Quaternion now(0, 0, yaw); // roll is around zaxis here actually!!!
        now.normalize();
        vir_odom.pose.pose.orientation.w = now.w();
        vir_odom.pose.pose.orientation.x = now.x();
        vir_odom.pose.pose.orientation.y = now.y();
        vir_odom.pose.pose.orientation.z = now.z();
        vir_odom_pub.publish(vir_odom);
        vir_speed_tmp.twist.twist.linear.x = v;
        vir_speed_tmp.twist.twist.angular.z = w;
        vir_speed_pub.publish(vir_speed_tmp);

        rate.sleep();
    }

    return 0;
}