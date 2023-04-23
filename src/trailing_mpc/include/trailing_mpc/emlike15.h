#ifndef EMLIKE15_H
#define EMLIKE15_H

#include "quadprog/Array.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include <queue>
#include <time.h>
#include "quadprog/QuadProg++.h"
#include "math.h"
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iterator>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "custom_messages/planning_info.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <std_msgs/Float64.h>

#include <proto/fm.pb.h>
#include <local_planning/cs15.h>
#include "custom_messages/status15.h"
#include "custom_messages/chief_cmd.h"
#include "custom_messages/track15.h"

#include "std_msgs/Float64.h"

#include "sys/stat.h"
// #include <cassert>
#include <iostream>

//longitude pid struct
struct pidKit
{
    double error_sum_;
    double last_error_;
    ros::Time last_time_;
    double last_speed_;
    pidKit(double error_sum, double last_error, ros::Time last_time, double last_speed):
        error_sum_(error_sum), last_error_(last_error), last_time_(last_time), last_speed_(last_speed)
    {}
};

class EMLike15{
public:

    EMLike15(ros::Publisher& vis_trajectory_pub, int recv_ID, int recv_target_ID){
        vis_trajectory_pub_ = vis_trajectory_pub;
        ID = recv_ID;
        target_ID = recv_target_ID;
    }
    ~EMLike15(){};

    void VisTrajectory();

    void ACallback(const custom_messages::status15::ConstPtr& p_msg);
    void BCallback(const custom_messages::status15::ConstPtr& p_msg);
    void CCallback(const custom_messages::status15::ConstPtr& p_msg);

    void refTrajCallback(const custom_messages::planning_info::ConstPtr& trajectory_msg);

    void Status15toCustomLoc(const custom_messages::status15::ConstPtr& input, custom_messages::vehicle_status& output);
    void LoadCustomLoc(custom_messages::vehicle_status& input, custom_messages::vehicle_status& output);

    void EMLike15Core();
    void EMLike15Prework();
    void ModifyHeadway();
    bool add_check();
    bool add_yaw_check();
    bool add_lateral_check();
    int findMatchpoint(double x, double y);
    double theat_angle();
    double theat_angle(double vx15, int traverse_i);
    std::array<double,5> cal_err_k();
    std::array<double,5> cal_err_k(double vx15, int traverse_i);
    std::array<double,2> forecast_position();
    bool cal_k(std::array<double,5> err_k, Eigen::Matrix<double,1,4>& output);
    bool cal_k(std::array<double,5> err_k, Eigen::Matrix<double,1,4>& output, double vx15, int traverse_i);
    bool cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double,4,1> B, Eigen::Matrix4d Q, Eigen::Matrix<double,1,1> R, Eigen::Matrix<double,1,4>& output);
    double cal_forward_angle(Eigen::Matrix<double,1,4> k,std::array<double,5> err_k);
    double cal_forward_angle(Eigen::Matrix<double,1,4> k,std::array<double,5> err_k, double vx15, int traverse_i);
    double cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle, std::array<double, 5> err_k);
    double speed_control_1();
    double speed_control_1(int traverse_i);
    double calc_speed(int mode);

    void fixpb4performance();
    void fixpb4lateral_error();
    void fixpb4yaw_error();
    void fixpb4follow_error();

    const std::string DataRecord();
    void DataRecordTool(double data1);

    // for rviz
    nav_msgs::Path visual_trajectory;
    ros::Publisher vis_trajectory_pub_;
    void GapPlot(double current_distance);

    int ID;
    int target_ID;
    int reconfiguration_flag; // 0-no, 1-add, 2-quit, 3-the add one. 4-after the add one one
    int last_reconfiguration_flag;
    ros::Time tick1; // quit start clock
    
    custom_messages::vehicle_status fmodom[3];
    custom_messages::vehicle_status front_pos;
    custom_messages::vehicle_status now_pos;

    pidKit kit1{0.0, 0.0, ros::Time::now(), 0.0};
    std::deque<int> add_keeper;

    int match_index;
    double vx;
    double vy = 0.0;

    CSRefPath ref_trajectory;

    //data iostream
    std::fstream file_to_write_;
};

#endif
