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
#include <local_planning/cubic_spline.h>
#include <std_msgs/Float64.h>
#include <proto/fm.pb.h>
#include <fstream>
#include <visualization_msgs/Marker.h>

// launch info
int ID;
double headway, current_headway, time_headway;
double vmin, vmax, wmin, wmax;  // 分别为v和w的控制量最小值最大值
double overall_length;
double emlike_hz;
double Kp, Ki, Kd;
double max_deceleration, max_acceleration;
double Kp2, Ki2, Kd2; // for other formation to use
double wheel_base;
double vy = 0.0;
double vx;
double cf, cr, m, a, b, Iz;
double forecast_time;
double Q1, Q2, Q3, Q4, lqrR;
int formation_type;
int controller_type;
double Kp1, Kd1; // controller2

// joint function
int match_index;
// reconfiguration function
std::vector<std::pair<int, nav_msgs::Odometry>> pList(10);
int reconfiguration_flag; // 0-no, 1-add, 2-quit, 3-the add one. 4-after the add one one
int last_reconfiguration_flag;
ros::Time tick1; // quit start clock
double reconfiguration_duration;
double add_lateral_threshold, add_yaw_threshold;
std::deque<int> add_keeper;

// ros related
custom_messages::vehicle_status front_pos;
custom_messages::vehicle_status leader_pos;
custom_messages::vehicle_status now_pos;
CSRefPath refTrajectory;

// for rviz
nav_msgs::Path Trajectory; 
ros::Publisher Vis_traj;

ros::Publisher cmd_pub;
// debug related
ros::Publisher gap_plot_pub;
std::ofstream tum_pose;

//boat
ros::Publisher virtual_environment_pub;

//longitude pid struct
struct pidKit
{
    double error_sum;
    double last_error;
    ros::Time last_time;
    double last_speed;
};

void visObstacle() {
    custom_messages::mapmodel vir_mapmodel;
    geometry_msgs::Point tmp;
    tmp.x = now_pos.xPos;
    tmp.y = now_pos.yPos;
    tmp.z = 0.8;
    vir_mapmodel.mapinfo.push_back(tmp);

    virtual_environment_pub.publish(vir_mapmodel);
}


int findMatchpoint(double x, double y);

bool add_lateral_check(custom_messages::vehicle_status &last_front) {
    if (abs(last_front.yaw - M_PI / 2) < 1e-6 || abs(last_front.yaw + M_PI / 2) < 1e-6) {
        double lateral_error = abs(last_front.xPos - front_pos.xPos);
        ROS_WARN("[em_like]: add_recfg lateral_error = %f(%f)", lateral_error, add_lateral_threshold);
        if (lateral_error > add_lateral_threshold) {
            return false;
        }
    } else {
        double k = tan(last_front.yaw);
        double A = k;
        double B = -1.0;
        double C = last_front.yPos - k * last_front.xPos;
        double x = front_pos.xPos;
        double y = front_pos.yPos;
        double lateral_d = sqrt(pow((A * x + B * y + C), 2)) / sqrt(pow(A, 2) + pow(B, 2));
        ROS_WARN("[em_like]: add_recfg lateral_error = %f(%f)", lateral_d, add_lateral_threshold);
        if (lateral_d > add_lateral_threshold) {
            return false;
        }
    }
    return true;
}
bool add_yaw_check(custom_messages::vehicle_status &last_front) {
    double yaw_error = abs(last_front.yaw - front_pos.yaw);
    ROS_WARN("[em_like]: add_recfg yaw_error = %f(%f)", yaw_error, add_yaw_threshold);
    if (yaw_error > add_yaw_threshold) {
        return false;
    } else {
        return true;
    }
}
bool add_check(custom_messages::vehicle_status &last_front) {
    while (add_keeper.size() > 5) {
        add_keeper.pop_front();
    }
    if (add_yaw_check(last_front) && add_lateral_check(last_front)) {
        add_keeper.push_back(1);
        return true;
    } else {
        add_keeper.push_back(0);
        return false;
    }
}

void formationCallback(const geometry_msgs::Point::ConstPtr& formation_msg){
    formation_type = formation_msg->x;
}

void ReconfigurationCallback(const geometry_msgs::Point::ConstPtr& reconstruction_msg){ // x: 1-add, 2-quit, y: ID
    int mode = reconstruction_msg->x;
    int reID = reconstruction_msg->y;
    if (mode == 1) {
        if (ID == 4) {
            ID = reID;
            reconfiguration_flag = 3;
        } else if (ID == reID) {
            ROS_WARN("[em_like]: I will gap up");
            reconfiguration_flag = 4;
            ++ID;
        } else if (ID > reID) {
            ++ID;
            reconfiguration_flag = 1;
        }
    } else if (mode == 2) {
        reconfiguration_flag = 2;
        if (ID == reID) {
            ros::shutdown();
        } else if (ID > reID) {
            --ID;
        }
    } else {
        ROS_WARN("[emlike]: wrong reconfiguration mode");
    }
}

bool GetVehicleInfo(int index, custom_messages::vehicle_status& out) {
    if (pList[index].first == 0) {
        // printf("[em_like]: get %i info failed\n", index);
        return false;
    }
    nav_msgs::Odometry tmp(pList[index].second);
    out.xPos = tmp.pose.pose.position.x;
    out.yPos = tmp.pose.pose.position.y;
    tf::Quaternion RQ2;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(tmp.pose.pose.orientation,RQ2);  
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  
    out.yaw = yaw;
    out.speed = tmp.twist.twist.linear.x;
    return true;
}

void ClearpList() {
    for (auto & x : pList) {
        x.first = 0;
    }
}

void Platoon0Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    int idx = p_msg->pose.pose.position.z;
    pList[idx].first = 1;
    pList[idx].second = *p_msg;
}
void Platoon1Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    int idx = p_msg->pose.pose.position.z;
    pList[idx].first = 1;
    pList[idx].second = *p_msg;
}
void Platoon2Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    int idx = p_msg->pose.pose.position.z;
    pList[idx].first = 1;
    pList[idx].second = *p_msg;
}
void Platoon3Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    int idx = p_msg->pose.pose.position.z;
    pList[idx].first = 1;
    pList[idx].second = *p_msg;
}
void Platoon4Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    int idx = p_msg->pose.pose.position.z;
    pList[idx].first = 1;
    pList[idx].second = *p_msg;
}

void refTrajCallback(const custom_messages::planning_info::ConstPtr& trajectory_msg){
    refTrajectory.CSclear();
    if (trajectory_msg->states.empty()) {
        return;
    }
    for(auto p : trajectory_msg->states){
        refTrajectory.r_curvature.emplace_back(p.angular_speed);
        refTrajectory.r_x.emplace_back(p.xPos);
        refTrajectory.r_y.emplace_back(p.yPos);
        refTrajectory.r_yaw.emplace_back(p.yaw);
        refTrajectory.r_s.emplace_back(p.speed);
    }
    printf("[em_like]: trajectory first s = %f, final s = %f\n", refTrajectory.r_s[0], refTrajectory.r_s.back());
}

void GapPlot(double error) {
    std_msgs::Float64 gap_plot_msg;
    gap_plot_msg.data = error;
    gap_plot_pub.publish(gap_plot_msg);
}

void ModifyHeadway() {
    custom_messages::vehicle_status last_front;
    // std::cout << "ID - 2 = " << ID - 2 << std::endl;
    ROS_FATAL("reconfiguration_flag = %i", reconfiguration_flag);
    ROS_FATAL("last_reconfiguration_flag = %i", last_reconfiguration_flag);
    GetVehicleInfo(ID - 2, last_front);
    // printf("[emlike]: last front info : x = %f, y = %f\n", last_front.xPos, last_front.yPos);
    if (reconfiguration_flag == 4) {
        current_headway = 3 * headway;
        ROS_WARN("[emlike]: add mode gap up:%f", current_headway);
    }
    if (last_reconfiguration_flag == 4) {
        add_check(last_front);
        int count = 0;
        for (auto x : add_keeper) {
            if (x == 1) {
                ++count;
            }
        }
        if (count >= 4) {
            current_headway = headway;
            ROS_WARN("[emlike]: add mode gap restore:%f", current_headway);
        }
    }

    if (reconfiguration_flag == 2) {
        tick1 = ros::Time::now();
        current_headway = 3 * headway;
        ROS_WARN("[emlike]: quit mode gap up:%f", current_headway);
    }
    if (last_reconfiguration_flag == 2) {
        ros::Time tock1 = ros::Time::now();
        double duration = tock1.toSec() - tick1.toSec();
        if (duration > reconfiguration_duration) {
            current_headway = headway;
            ROS_WARN("[emlike]: quit mode gap restore:%f", current_headway);
        }
    }
}

double speed_control_1(pidKit& kit1) {
    // dd=d_min; // 1.constant
    // dd=d_min+h_s*myselfcar.Velocity.v; // 2.+speed
    // dd=d_min+h_s*myselfcar.Velocity.v+G_s*std::pow(myselfcar.Velocity.v,2); // 3.+a

    double current_gap = refTrajectory.r_s.back() - refTrajectory.r_s[match_index];
    double pid_speed;
    double euclidian_distance = sqrt(pow(front_pos.xPos - now_pos.xPos, 2) + pow(front_pos.yPos - now_pos.yPos, 2));
    if(!formation_type){
        double error = current_gap - current_headway;
        // double dt = ros::Time::now().toSec() - kit1.last_time.toSec();
        // kit1.error_sum += error * dt;
        // double derivative = (error - kit1.last_error) / dt;
        // pid_speed = front_pos.speed + Kp * error + Ki * kit1.error_sum + Kd * derivative;
        pid_speed = front_pos.speed + Kp * error;
        // kit1.last_time = ros::Time::now();
        // kit1.last_error = error;

        std::cout << "[emlike]: current_headway : "<< current_headway << "(" << headway << ")" << ", current_gap : " << current_gap << ", error = " << error << "(euclidian distance = " << euclidian_distance << ")" << std::endl;
        std::cout << "[emlike]: front speed : "<< front_pos.speed << ", ego speed : " << now_pos.speed << ", raw pid_speed = " << pid_speed << std::endl;
        GapPlot(error);
        tum_pose << std::fixed << std::setprecision(3) << error << " ";
    }
    else{
        double error = current_gap - overall_length;
        pid_speed = leader_pos.speed + Kp2 * error;
        std::cout << "[emlike]: leaderspeed: " << leader_pos.speed << ", Kp2*error: " << Kp2 * error << std::endl;
        std::cout << "[emlike]: target x: "<< refTrajectory.r_x.back() << ", y: " << refTrajectory.r_y.back() << ", current_gap= " << current_gap << ", error=" << error << std::endl;
        std::cout << "[emlike]: leader speed : "<< leader_pos.speed << ", ego speed : " << now_pos.speed << ", raw pid_speed = " << pid_speed << std::endl;
        GapPlot(error);
        tum_pose << std::fixed << std::setprecision(3) << error << " ";
    }

    // limiter 1 : can not speed up or down too fast
    if (pid_speed < kit1.last_speed - max_deceleration) {
        pid_speed = kit1.last_speed - max_deceleration;
    } else if (pid_speed > kit1.last_speed + max_acceleration) {
        pid_speed = kit1.last_speed + max_acceleration;
    }
    // limiter 2 : can not back
    if (pid_speed < vmin) {
        pid_speed = vmin;
    } else if (pid_speed > 2.0 * vmax) {
        pid_speed = 2.0 * vmax;
    }

    kit1.last_speed = pid_speed;
    return pid_speed;
}

double speed_control_2(pidKit& kit2) {
    double current_gap = refTrajectory.r_s.back() - refTrajectory.r_s[match_index];
    std::cout << "frenet_s = " << refTrajectory.r_s[match_index] << "final s = " << refTrajectory.r_s.back() << std::endl;
    double output_speed, control_error, control_error_d;
    if(!formation_type){
        control_error = current_gap - current_headway;
        control_error_d = front_pos.speed - now_pos.speed;
        output_speed = kit2.last_speed + Kp1 * control_error + Kd1 * control_error_d;

        double euclidian_distance = sqrt(pow(front_pos.xPos - now_pos.xPos, 2) + pow(front_pos.yPos - now_pos.yPos, 2));
        std::cout << "headway : "<< current_headway << ", current_gap : " << current_gap << ", error = " << control_error  << "(euclidian distance = " << euclidian_distance << ")" << std::endl;
        std::cout << "front speed : "<< front_pos.speed << ", ego speed : " << now_pos.speed << ", raw output_speed = " << output_speed << std::endl;
    }

    // limiter 1 : can not speed up or down too fast
    if (output_speed < kit2.last_speed - max_deceleration) {
        output_speed = kit2.last_speed - max_deceleration;
    } else if (output_speed > kit2.last_speed + max_acceleration) {
        output_speed = kit2.last_speed + max_acceleration;
    }
    // limiter 2 : can not break upper/lower bound
    if (output_speed < vmin) {
        output_speed = vmin;
    } else if (output_speed > 2 * vmax) {
        output_speed = 2 * vmax;
    }

    kit2.last_speed = now_pos.speed;
    return output_speed;
}

double calc_speed(int mode, pidKit& kit1, pidKit& kit2) {
    if (mode == 1) {
        return speed_control_1(kit1);
    } else if (mode == 2) {
        return speed_control_2(kit2);
    }
    std::cout << "[emlike]: mode error" << std::endl;
    return -1;
}

std::array<double,2> forecast_position() {
    std::array<double,2> position;
    position[0] = now_pos.xPos + (now_pos.speed) * forecast_time * cos(now_pos.yaw);
    position[1] = now_pos.yPos + (now_pos.speed) * forecast_time * sin(now_pos.yaw);
    return position;
}

int findMatchpoint(double x, double y){
    int num = refTrajectory.r_curvature.size();
    double dis_min = std::numeric_limits<double>::max();
    int index = 0;
    for (int i = 0; i < num; ++i) {
        double temp_dis = std::pow(refTrajectory.r_x[i] - x, 2) + std::pow(refTrajectory.r_y[i] - y, 2);
        //ROS_ERROR("%s,%f.%s,%f","r_x",r_x[i],"r_y",r_y[i]);
        //ROS_ERROR("%s,%f","distance",temp_dis);
        if (temp_dis < dis_min) {
            dis_min = temp_dis;
            index = i;
        }
    }
    // ROS_ERROR("%s,%i","nearest point",index);
    return index;
}

std::array<double,5> cal_err_k(){
    std::array<double,5> err_k;
    int target_index = match_index;

    //forecast, if small it will not work, if big it will overshoot
    std::array<double,2> forecast_pos = forecast_position();
    int forecast_index = findMatchpoint(forecast_pos[0], forecast_pos[1]);
    target_index = forecast_index;

    // direction vector
    Eigen::Matrix<double,2,1> tor, nor;
    tor << cos(refTrajectory.r_yaw[target_index]), sin(refTrajectory.r_yaw[target_index]);
    nor << -sin(refTrajectory.r_yaw[target_index]), cos(refTrajectory.r_yaw[target_index]);

    // cartesian error
    Eigen::Matrix<double,2,1> d_err;
    d_err << now_pos.xPos - refTrajectory.r_x[target_index], now_pos.yPos - refTrajectory.r_y[target_index];
    double phi = now_pos.yaw;
    double ed = nor.transpose() * d_err;
    double es = tor.transpose() * d_err;
    double projection_point_theta = refTrajectory.r_yaw[target_index] + refTrajectory.r_curvature[target_index] * es;
    double ed_d = vy * std::cos(phi - projection_point_theta) + vx * std::sin(phi - projection_point_theta);
    double ephi = phi - projection_point_theta;
    double s_d = (vx * std::cos(phi - projection_point_theta) -  vy * std::sin(phi - projection_point_theta)) /  (1 - refTrajectory.r_curvature[target_index] * ed);
    double phi_d = vx * refTrajectory.r_curvature[target_index];
    double ephi_d = phi_d - refTrajectory.r_curvature[target_index] * s_d;
    err_k[0] = ed;
    err_k[1] = ed_d;
    err_k[2] = ephi;
    err_k[3] = ephi_d;
    err_k[4] = refTrajectory.r_curvature[target_index];
    if(isnan(err_k[0])){
        ROS_WARN("[emlike]: err0 error!!!");
        while(1);        
    }
    if(isnan(err_k[1])){
        ROS_WARN("[emlike]: err1 error!!!");
        while(1);        
    }
    if(isnan(err_k[2])){
        ROS_WARN("[emlike]: err2 error!!!");
        while(1);        
    }
    if(isnan(err_k[3])){
        ROS_WARN("[emlike]: err3 error!!!");
        while(1);        
    }

    return err_k;
}

/**
 * @brief 迭代法求黎卡提方程
 * @param A 线性化状态量项系数
 * @param B 线性化控制量项系数
 * @param Q 状态量权重
 * @param R 控制量权重
 * @return k 反馈控制项系数 
 */
bool cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double,4,1> B, Eigen::Matrix4d Q, Eigen::Matrix<double,1,1> R, Eigen::Matrix<double,1,4>& output){
    //设置最大循环迭代次数
    int numLoop = 200;
    //设置目标极小值
    double minValue = 10e-10;
    Eigen::Matrix4d p_old;
    p_old = Q;
    //离散化方程
    double ts = 0.001;
    Eigen::Matrix4d eye,Ad;
    Eigen::Matrix<double,4,1> Bd;
    eye.setIdentity(4,4);
    Ad = (eye - ts * 0.5 * A).inverse() * (eye + ts * 0.5 * A);
    Bd = B * ts;
    /************开始迭代****************/
    for (int i = 0; i < numLoop; ++i) {
        Eigen::Matrix4d p_new = Ad.transpose() * p_old * Ad - Ad.transpose() * p_old * Bd *(R + Bd.transpose() * p_old * Bd).inverse() *Bd.transpose() * p_old * Ad + Q;
        // if (isnan(p_new(0)) || isnan(p_new(1)) || isnan(p_new(2)) || isnan(p_new(3)) ) {
        //     std::cout << p_old << std::endl;
        //     std::cout << Ad << std::endl;
        //     std::cout << Ad.transpose() << std::endl;
        //     std::cout << Bd << std::endl;
        //     std::cout << Bd.transpose() << std::endl;
        //     std::cout << R << std::endl;
        //     std::cout << Q << std::endl;
        //     Eigen::MatrixXd test_1 = (R + Bd.transpose() * p_old * Bd);
        //     if (isnan(test_1(0)) || isnan(test_1(1)) || isnan(test_1(2)) || isnan(test_1(3)) ) {
        //         std::cout << "test_1 nan" << std::endl;
        //     } else {
        //         std::cout << test_1 << std::endl;
        //     }
        //     Eigen::MatrixXd test_2 = (R + Bd.transpose() * p_old * Bd).inverse();
        //     if (isnan(test_2(0)) || isnan(test_2(1)) || isnan(test_2(2)) || isnan(test_2(3)) ) {
        //         std::cout << "test_2 nan" << std::endl;
        //     } else {
        //         std::cout << test_2 << std::endl;
        //     }
        //     while (1) {

        //     }
        // }
        p_old = p_new;
        if (fabs((p_new - p_old).maxCoeff()) < minValue) {
            break;
        }

    }
    if(isnan(p_old(0))){
        ROS_WARN("[emlike]: p0 error!!!");
        return false;   
    }
    if(isnan(p_old(1))){
        ROS_WARN("[emlike]: p1 error!!!");
        return false;        
    }
    if(isnan(p_old(2))){
        ROS_WARN("[emlike]: p2 error!!!");
        return false;       
    }
    if(isnan(p_old(3))){
        ROS_WARN("[emlike]: p3 error!!!");
        return false;        
    }
    output = (R + Bd.transpose() * p_old * Bd).inverse() * Bd.transpose() * p_old * Ad;
    if(isnan(output[0])){
        ROS_WARN("[emlike]: k0 error!!!");
        return false;       
    }
    if(isnan(output[1])){
        ROS_WARN("[emlike]: k1 error!!!");
        return false;       
    }
    if(isnan(output[2])){
        ROS_WARN("[emlike]: k2 error!!!");
        return false;         
    }
    if(isnan(output[3])){
        ROS_WARN("[emlike]: k3 error!!!");
        return false;        
    }
    return true;
}

bool cal_k(std::array<double,5> err_k, Eigen::Matrix<double,1,4>& output){
    Eigen::Matrix4d A;
    Eigen::Matrix<double,4,1> B;
    Eigen::Matrix4d Q;
    Eigen::Matrix<double,1,1> R;
    A<<   0,            1,                                              0,                                0,
            0,           (cf+cr)/(m*vx),                  -(cf+cr)/m,                (a*cf-b*cr)/(m*vx),
            0,            0,                                              0,                                1,
            0,           (a*cf-b*cr)/(Iz*vx),           -(a*cf-b*cr)/Iz,         (a*a*cf+b*b*cr)/(Iz*vx);
    B<<   0,          -cf/m,                                        0,                               -a*cf/Iz;
    Q=Eigen::Matrix4d::Zero(4,4);
    Q(0,0) = Q1;
    Q(1,1) = Q2;
    Q(2,2) = Q3;
    Q(3,3) = Q4;
    R(0,0) = lqrR;
    if (!cal_dlqr(A, B, Q, R, output)) {
        return false;
    }
    return true;
}

double cal_forward_angle(Eigen::Matrix<double,1,4> k,std::array<double,5> err_k){
    double k3 = k[2];
    // 不足转向系数
    double kv = b * m / (cf * wheel_base) - a * m / (cr * wheel_base);
    double point_curvature = err_k[4];
    double forword_angle =  wheel_base * point_curvature + kv * vx * vx * point_curvature - k3 * (b * point_curvature - a * m * vx * vx * point_curvature / cr / b);
    if(isnan(forword_angle)){
        ROS_WARN("[emlike]: forword_angle error!!!");
        while(1);        
    }
    return forword_angle;
}

double cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle, std::array<double, 5> err_k){
    Eigen::Matrix<double, 4, 1> err;
    err << err_k[0], err_k[1], err_k[2], err_k[3];
    double angle = -k * err + forword_angle;
    if(isnan(angle)){
        ROS_WARN("[emlike]: angle error!!!");
        while(1);        
    }
    return angle;
}

double theat_angle(){
    // error state space model
    std::array<double,5> err_k = cal_err_k();
    std::cout << "[em_like]: error : " << err_k[0] << "," << err_k[1] << "," << err_k[2] << "," << err_k[3] << "," << err_k[4] << "," << std::endl;
    // u = -kx + deltaf : k
    Eigen::Matrix<double,1,4> k;
    if (!cal_k(err_k, k)) {
        return 0.0;
    }
    std::cout << "[em_like]: k=" << k << std::endl;
    // u = -kx + deltaf : deltaf
    double forword_angle = cal_forward_angle(k,err_k);
    std::cout << "[em_like]: feedforward=" << forword_angle << std::endl;
    // u = -kx + deltaf
    double lqr_delta = cal_angle(k, forword_angle, err_k);
    std::cout << "[em_like]: raw_lqr_delta=" << lqr_delta << std::endl;
    if (lqr_delta > wmax) {
        lqr_delta = wmax;
    } else if (lqr_delta < wmin) {
        lqr_delta = wmin;
    }

    return lqr_delta;
}

void EM(pidKit& kit1, pidKit& kit2){
    if(refTrajectory.r_s.empty()){
        ROS_WARN("[emlike]: NO trajectory of spline");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_pub.publish(cmd_vel);
        return;
    }

    tum_pose << std::fixed << std::setprecision(3) << ros::Time::now().toSec() << " ";

    if (GetVehicleInfo(0, leader_pos)) {
        // return;
        // printf("[emlike]: LEADER X = %f, Y = %f\n", leader_pos.xPos, leader_pos.yPos);
    }
    if (GetVehicleInfo(ID - 1, front_pos)) {
        // return;
        printf("[emlike]: FRONT X = %f, Y = %f\n", front_pos.xPos, front_pos.yPos);
    }

    // quit mode modify headway
    if (ID > 1) {
        ModifyHeadway();
    }

    // reconfiguration skip
    if (reconfiguration_flag == 4) {
        return;
    }

    Trajectory.header.stamp = ros::Time::now();
    Trajectory.header.frame_id = "map";
    geometry_msgs::PoseStamped TrajectoryPoint;
    TrajectoryPoint.header.frame_id="map";
    TrajectoryPoint.pose.position.x=now_pos.xPos;
    TrajectoryPoint.pose.position.y=now_pos.yPos;
    Trajectory.poses.emplace_back(TrajectoryPoint);
    Vis_traj.publish(Trajectory);

    match_index = findMatchpoint(now_pos.xPos, now_pos.yPos);
    // std::cout << "[em_like]: match_index:" << match_index << std::endl;

    double speed_final = calc_speed(controller_type, kit1, kit2);
    double delta_final, aspeed_final;

    if (vx < 1e-4) {
        aspeed_final = 0.0;
    } else {
        delta_final = theat_angle();
        // ackerman to differential
        aspeed_final = speed_final * std::tan(delta_final) / wheel_base;
    }

    tum_pose << std::fixed << std::setprecision(3) << speed_final << " " << aspeed_final << " ";

    // double w_ratio = abs(aspeed_final / wmax);
    // double w_penalty = w_ratio * 0.2;
    // aspeed_final *= w_penalty;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = speed_final;
    cmd_vel.angular.z = aspeed_final;
    cmd_pub.publish(cmd_vel);
    std::cout << "[em_like]: control_output: v = " << speed_final << ", w = " << aspeed_final << std::endl;

    tum_pose << std::fixed << std::setprecision(3) << now_pos.xPos << " " << now_pos.yPos << std::endl;
}

int main(int argc, char  **argv)
{
    ros::init(argc,argv,"EMcontroller_node");
    ros::NodeHandle nh;

    nh.param<int>("ID", ID, 0);
    nh.param<double>("overall_length", overall_length, 0.6);
    nh.param<double>("wheel_base", wheel_base, 0.48);
    nh.param<double>("cf", cf, -115494663);
    nh.param<double>("cr", cr, -115494663);
    nh.param<double>("weight", m, 20.0);
    nh.param<double>("a", a, 0.24);
    nh.param<double>("b", b, 0.24);
    Iz = std::pow(a, 2) * m/2 + std::pow(b, 2) * m/2; // double Iz = std::pow(a, 2) * mass_front + std::pow(b, 2) * mass_rear;
    nh.param<int>("/formation_type", formation_type, 0);
    nh.param<int>("controller_type", controller_type, 1);

    nh.getParam("add_lateral_threshold", add_lateral_threshold);
    nh.getParam("add_yaw_threshold", add_yaw_threshold);
    nh.getParam("vmin", vmin);
    nh.getParam("vmax", vmax);
    nh.getParam("wmax", wmax);
    wmin = -wmax;
    nh.getParam("headway", headway);
    current_headway = headway;
    nh.getParam("time_headway", time_headway);
    nh.getParam("Kp", Kp);
    nh.getParam("Ki", Ki);
    nh.getParam("Kd", Kd);
    nh.getParam("max_deceleration", max_deceleration);
    nh.getParam("max_acceleration", max_acceleration);
    nh.getParam("Kp2", Kp2);
    nh.getParam("Ki2", Ki2);
    nh.getParam("Kd2", Kd2);
    nh.getParam("Kp1", Kp1);
    nh.getParam("Kd1", Kd1);
    nh.getParam("lqr_q1", Q1);
    nh.getParam("lqr_q2", Q2);
    nh.getParam("lqr_q3", Q3);
    nh.getParam("lqr_q4", Q4);
    nh.getParam("lqr_r", lqrR);
    nh.getParam("forecast_time", forecast_time);
    nh.getParam("reconfiguration_duration", reconfiguration_duration);
    nh.getParam("emlike_hz", emlike_hz);

    Vis_traj = nh.advertise<nav_msgs::Path>("VisTraj", 100);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    gap_plot_pub = nh.advertise<std_msgs::Float64>("gap", 20);
    virtual_environment_pub = nh.advertise<custom_messages::mapmodel>("/No_0/virtual_obstacle_info", 1);

    ros::Subscriber formation_sub = nh.subscribe("/formationType", 1, formationCallback);
    ros::Subscriber reconfiguration_sub = nh.subscribe("/reID", 1, ReconfigurationCallback);

    ros::Subscriber platoon0_sub = nh.subscribe("/No_0/myodom", 5, Platoon0Callback);
    ros::Subscriber platoon1_sub = nh.subscribe("/No_1/myodom", 5, Platoon1Callback);
    ros::Subscriber platoon2_sub = nh.subscribe("/No_2/myodom", 5, Platoon2Callback);
    ros::Subscriber platoon3_sub = nh.subscribe("/No_3/myodom", 5, Platoon3Callback);
    ros::Subscriber platoon4_sub = nh.subscribe("/No_4/myodom", 5, Platoon4Callback);
    
    ros::Subscriber mypathpose = nh.subscribe("trajectory_cubic_spline", 10, refTrajCallback);

    std::string myFileName = " ";
    myFileName = "shiyan_" + std::to_string(ID) + ".txt";//存储数据的文件名
    tum_pose.open(ROOT_DIR + myFileName);
    if (!tum_pose.is_open())
    {
        std::cout << "文件打开失败" << std::endl;
        return 0;
    }

    pidKit kit1{0.0, 0.0, ros::Time::now(), 0.0};
    pidKit kit2{0.0, 0.0, ros::Time::now(), 0.0};
    max_deceleration /= emlike_hz;
    max_acceleration /= emlike_hz;
    
    ros::Rate loop_rate(emlike_hz);
    while (ros::ok()) {
        reconfiguration_flag = 0;

        ros::spinOnce();

        // boat
        // visObstacle();

        std::cout << "[em_like] node ID = " << ID << std::endl;

        if (GetVehicleInfo(ID, now_pos)) {
            std::cout << "[emlike]: myX:" << now_pos.xPos << ", myY: " << now_pos.yPos << std::endl;
        }
        vx = now_pos.speed;

        ros::Time time1 = ros::Time::now();
        EM(kit1, kit2);
        ros::Time time2 = ros::Time::now();
        if (reconfiguration_flag != 0) {
            last_reconfiguration_flag = reconfiguration_flag;
        }
        std::printf("[em_like]: Time consume is : %f\n",(time2 - time1).toSec() );

        loop_rate.sleep();
    }
    tum_pose.close();
    return 0;
}