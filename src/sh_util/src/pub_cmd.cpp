#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pub_cmd");
    ros::NodeHandle n;

    ros::Publisher test_cmd_pub = n.advertise<geometry_msgs::Twist>("/No_0/cmd_vel", 1);

    double max_speed = 3.0;
    double start_time = ros::Time::now().toSec();
    double hz = 50.0;

    ros::Rate loop_rate(hz);
    while (ros::ok())
    {
        geometry_msgs::Twist test_cmd;
        double now_time = ros::Time::now().toSec();
        double time_cut = now_time - start_time;
        if (time_cut < 15.0) {
            test_cmd.linear.x = 0.2 * time_cut;
        } else if (time_cut > 25.0 && time_cut < 40.0) {
            test_cmd.linear.x = max_speed - 0.2 * (time_cut - 25.0);
        } else if (time_cut > 15.0 && time_cut < 25.0) {
            test_cmd.linear.x = max_speed;
        } else if (time_cut > 40.0) {
            test_cmd.linear.x = 0.0;
        }
        std::cout << "[" << time_cut << "]" << test_cmd.linear.x << std::endl;
        test_cmd_pub.publish(test_cmd);
        loop_rate.sleep();
    }
 
    return 0;
}