#ifndef MONITOR15_H
#define MONITOR15_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

#include <custom_messages/planning_info.h>
#include <custom_messages/vehicle_status.h>

#include "custom_messages/status15.h"
#include "custom_messages/chief_cmd.h"
#include "include/proto/fm.pb.h"

#include <arpa/inet.h>

#include "sh_util/sh_util.h"

namespace fm = yw::v2xclient::proto;

class Monitor15
{
public:
    Monitor15() {
        mList.resize(3);
        for (auto x : mList) {
            x.first = false;
        }
    }

    void ACallback(const custom_messages::status15::ConstPtr& p_msg);
    void BCallback(const custom_messages::status15::ConstPtr& p_msg);
    void CCallback(const custom_messages::status15::ConstPtr& p_msg);

    void CTargetCallback(const std_msgs::Int32::ConstPtr& p_msg);
    void CFollowdistanceCallback(const std_msgs::Float64::ConstPtr& p_msg);

    void run();
    bool start_check();
    void Status15toCustomLoc(custom_messages::status15& input, custom_messages::vehicle_status& output);
    void BroadcastChiefcmd();
    void BroadcastChiefcmdFakeTest();
    bool online_check();

    std::vector<std::pair<bool, custom_messages::status15>> mList;
    ros::Publisher _start_check_pub;

    int C_target_;
    bool get_Ctarget_flag = false;
    double C_followdistance_;
    bool get_Cfollowdistance_flag = false;
    bool start_check_button_;
};

#endif // MONITOR15_H
