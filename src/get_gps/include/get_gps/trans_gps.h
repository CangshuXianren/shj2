/************************************************
@jyf
功能包名称 ： trans_gps
函数功能 ： 订阅自身gps信息并通过uwb广播消息
备注 ： 
*************************************************/

#ifndef TRANSGPS_H
#define TRANSGPS_H

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include <string>
#include "std_msgs/Float64.h"
#include <unistd.h>
#include "custom_messages/msgNav982Pkt20.h"
//#include "novatel_oem7_msgs/INSPVAX.h"

#define GPSMSGTYPE custom_messages::msgNav982Pkt20


class Trans_GPS  
{  
public:  
    /*Constructor:
     *pub     Publisher,发送gps信息给后车
     */
    Trans_GPS(ros::Publisher pub);

    ~Trans_GPS();

    /*
     * novatel中的经纬度和方位角都是角度
     */
    void gpsfixCallback(const GPSMSGTYPE::ConstPtr& gps_msg);

//变量
    ros::Publisher pub;
    
};

#endif
