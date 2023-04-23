#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "custom_messages/mapmodel.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_environment");
    ros::NodeHandle nh;

    ros::Publisher virtual_environment_pub = nh.advertise<custom_messages::mapmodel>("virtual_obstacle_info", 1);

    ros::Rate rate(100);
    while(ros::ok())
    {
        custom_messages::mapmodel vir_mapmodel;
        geometry_msgs::Point tmp;

            // boat
        // for (int i = 0; i < 10; ++i) {
        //     tmp.x = 0.0 + 0.2 * i;
        //     tmp.y = -80.0;
        //     tmp.z = 1.0;
        //     vir_mapmodel.mapinfo.push_back(tmp);
        // }
        // for (int i = 0; i < 10; ++i) {
        //     tmp.x = 0.0 + 0.2 * i;
        //     tmp.y = -85.0;
        //     tmp.z = 1.0;
        //     vir_mapmodel.mapinfo.push_back(tmp); 
        // }
        // tmp.x = 5.0;
        // tmp.y = -82.0;
        // tmp.z = 0.7;
        // vir_mapmodel.mapinfo.push_back(tmp);
        // tmp.x = -7.5;
        // tmp.y = -87.0;
        // tmp.z = 0.7;
        // vir_mapmodel.mapinfo.push_back(tmp);
        // tmp.x = -7.5;
        // tmp.y = -84.0;
        // tmp.z = 0.7;
        // vir_mapmodel.mapinfo.push_back(tmp);
        // tmp.x = -7.5;
        // tmp.y = -81.0;
        // tmp.z = 0.7;
        // vir_mapmodel.mapinfo.push_back(tmp);

        tmp.x = 5.0;
        tmp.y = -82.0;
        tmp.z = 15.0;
        vir_mapmodel.mapinfo.push_back(tmp);

        virtual_environment_pub.publish(vir_mapmodel);
        rate.sleep();
    }

    return 0;
}