#ifndef MONITOR_H
#define MONITOR_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

#include <custom_messages/planning_info.h>
#include <custom_messages/vehicle_status.h>


class Monitor
{
public:
    Monitor(ros::Publisher& start_check_pub);
    Monitor();
    void Platoon0Callback(const nav_msgs::Odometry::ConstPtr& p_msg);
    void Platoon1Callback(const nav_msgs::Odometry::ConstPtr& p_msg);
    void Platoon2Callback(const nav_msgs::Odometry::ConstPtr& p_msg);
    void Platoon3Callback(const nav_msgs::Odometry::ConstPtr& p_msg);
    void run();
    bool start_check();
    bool online_check();
    bool GetVehicleInfo(int index, custom_messages::vehicle_status& out);

    std::vector<std::pair<int, nav_msgs::Odometry>> mList;
    ros::Publisher _start_check_pub;
};

#endif // MONITOR_H
