#ifndef BRIDGE_H
#define BRIDGE_H

#include <vector>
#include <iostream>
#include <algorithm>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

#include <custom_messages/chief_cmd.h>
#include <custom_messages/status15.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include "include/proto/fm.pb.h"

namespace fm = yw::v2xclient::proto;

class Bridge
{
public:
    Bridge(){};
    ~Bridge(){};

    void init(unsigned short& port, int& sock);
    void run(int& sock, char (&buffer)[1024]);
    void fm2status15(fm::FormationMsg& input, custom_messages::status15& output);
    void fm2chiefcmd(fm::FormationMsg& input, custom_messages::chief_cmd& output);
    void Status15toRosodometry(custom_messages::status15& input, nav_msgs::Odometry& output);

    ros::Publisher number1_pub;
    ros::Publisher number2_pub;
    ros::Publisher number3_pub;
    ros::Publisher chief_pub;

    ros::Publisher number1_vis_pub;
    ros::Publisher number2_vis_pub;
    ros::Publisher number3_vis_pub;

    bool log_mode_ = false;
};

#endif // BRIDGE_H
