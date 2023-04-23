#include "trailing_mpc/emlike15.h"

// launch info
double headway, current_headway, max_headway;
double vmin, vmax, wmin, wmax;  // 分别为v和w的控制量最小值最大值
double overall_length;
double emlike15_hz;
double Kp, Ki, Kd;
double max_deceleration, max_acceleration;
double wheel_base;
double cf, cr, m, a, b, Iz;
double forecast_time;
double Q1, Q2, Q3, Q4, lqrR;
int formation_type;
double time_headway;
double merge_lateral_threshold, merge_yaw_threshold;
double reconfiguration_duration;

// ros related
ros::Publisher cmd_pub, C_followdistance_pub, full_trajectory_pub, lateral_error_pub, yaw_error_pub, follow_error_pub;
// debug related
ros::Publisher gap_plot_pub, current_gap_pub, ideal_gap_pub;

void EMLike15::LoadCustomLoc(custom_messages::vehicle_status& input, custom_messages::vehicle_status& output) {
    output.xPos = input.xPos;
    output.yPos = input.yPos;
    output.yaw = input.yaw;
    output.speed = input.speed;
    output.angular_speed = input.angular_speed;
}

void EMLike15::Status15toCustomLoc(const custom_messages::status15::ConstPtr& input, custom_messages::vehicle_status& output) {
    output.xPos = input->position.x;
    output.yPos = input->position.y;
    output.yaw = input->pose.yaw;
    output.speed = input->linear_velocity.x;
    output.angular_speed = input->angular_velocity.z;
}

void EMLike15::EMLike15Prework() {
    // emlike related
    LoadCustomLoc(fmodom[ID], now_pos);
    VisTrajectory();
    vx = now_pos.speed;

    // //data record
    // std::string file_path = DataRecord();
    // std::ofstream f;
    // f.open(file_path, std::ios::trunc);
    // DataRecordTool(ros::Time::now().toSec());
}

void EMLike15::VisTrajectory() {
    visual_trajectory.header.stamp = ros::Time::now();
    visual_trajectory.header.frame_id = "map";
    geometry_msgs::PoseStamped TrajectoryPoint;
    TrajectoryPoint.header.frame_id = "map";
    TrajectoryPoint.pose.position.x = now_pos.xPos;
    TrajectoryPoint.pose.position.y = now_pos.yPos;
    visual_trajectory.poses.emplace_back(TrajectoryPoint);
    vis_trajectory_pub_.publish(visual_trajectory);
}

bool EMLike15::add_lateral_check() {
    if (abs(fmodom[0].yaw - M_PI / 2) < 1e-6 || abs(fmodom[0].yaw + M_PI / 2) < 1e-6) {
        double lateral_error = abs(fmodom[0].xPos - fmodom[1].xPos);
        ROS_WARN("[emlike15]: add_recfg lateral_error = %f(%f)", lateral_error, merge_lateral_threshold);
        if (lateral_error > merge_lateral_threshold) {
            return false;
        }
    } else {
        double k = tan(fmodom[0].yaw);
        double A = k;
        double B = -1.0;
        double C = fmodom[0].yPos - k * fmodom[0].xPos;
        double x = fmodom[1].xPos;
        double y = fmodom[1].yPos;
        double lateral_d = sqrt(pow((A * x + B * y + C), 2)) / sqrt(pow(A, 2) + pow(B, 2));
        ROS_WARN("[emlike15]: add_recfg lateral_error = %f(%f)", lateral_d, merge_lateral_threshold);
        if (lateral_d > merge_lateral_threshold) {
            return false;
        }
    }
    return true;
}
bool EMLike15::add_yaw_check() {
    double yaw_error = abs(fmodom[0].yaw - fmodom[1].yaw);
    ROS_WARN("[emlike15]: add_recfg yaw_error = %f(%f)", yaw_error, merge_yaw_threshold);
    if (yaw_error > merge_yaw_threshold) {
        return false;
    } else {
        return true;
    }
}
bool EMLike15::add_check() {
    while (add_keeper.size() > 5) {
        add_keeper.pop_front();
    }
    if (add_yaw_check() && add_lateral_check()) {
        add_keeper.push_back(1);
        return true;
    } else {
        add_keeper.push_back(0);
        return false;
    }
}

void EMLike15::ACallback(const custom_messages::status15::ConstPtr& p_msg){
    std::cout << "[emlike15]: get A loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    Status15toCustomLoc(p_msg, fmodom[0]);
}
void EMLike15::BCallback(const custom_messages::status15::ConstPtr& p_msg){
    std::cout << "[emlike15]: get B loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    Status15toCustomLoc(p_msg, fmodom[1]);
    if (p_msg->request == 2) {
        if (ID == 2) {
            reconfiguration_flag = 4;
            target_ID = 1;
        }
    } else if (p_msg->request == 1) {
        if (ID == 2) {
            target_ID = 0;
            reconfiguration_flag = 2;
        }
    }
}
void EMLike15::CCallback(const custom_messages::status15::ConstPtr& p_msg){
    std::cout << "[emlike15]: get C loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    Status15toCustomLoc(p_msg, fmodom[2]);
}

void EMLike15::refTrajCallback(const custom_messages::planning_info::ConstPtr& trajectory_msg){
    ref_trajectory.CSclear();
    if (trajectory_msg->states.empty()) {
        return;
    }
    for(auto p : trajectory_msg->states){
        ref_trajectory.r_curvature.emplace_back(p.angular_speed);
        ref_trajectory.r_x.emplace_back(p.xPos);
        ref_trajectory.r_y.emplace_back(p.yPos);
        ref_trajectory.r_yaw.emplace_back(p.yaw);
        ref_trajectory.r_s.emplace_back(p.speed);
    }
    printf("[emlike15]: trajectory first s = %f, final s = %f\n", ref_trajectory.r_s[0], ref_trajectory.r_s.back());
}

void EMLike15::GapPlot(double current_distance) {
    std_msgs::Float64 gap_plot_msg;
    gap_plot_msg.data = current_distance - current_headway;
    gap_plot_pub.publish(gap_plot_msg);
    gap_plot_msg.data = current_distance;
    current_gap_pub.publish(gap_plot_msg);
    gap_plot_msg.data = current_headway;
    ideal_gap_pub.publish(gap_plot_msg);
}

void EMLike15::ModifyHeadway() {
    // printf("last front info : x = %f, y = %f\n", last_front.xPos, last_front.yPos);
    std::cout << "reconfiguration_flag:" << reconfiguration_flag << ", last_reconfiguration_flag:" << last_reconfiguration_flag << std::endl;
    if (reconfiguration_flag == 4) {
        current_headway = 3 * headway;
        ROS_WARN("current headway up(add mode):%f", current_headway);
    }
    if (last_reconfiguration_flag == 4) {
        add_check();
        int count = 0;
        for (auto x : add_keeper) {
            if (x == 1) {
                ++count;
            }
        }
        if (count >= 4) {
            current_headway = headway;
            ROS_WARN("current headway restore(add mode):%f", current_headway);
            last_reconfiguration_flag = 0;
        }
    }

    if (reconfiguration_flag == 2) {
        tick1 = ros::Time::now();
        current_headway = 3 * headway;
        ROS_WARN("current headway up to(quit mode):%f", current_headway);
    }
    if (last_reconfiguration_flag == 2) {
        ros::Time tock1 = ros::Time::now();
        double duration = tock1.toSec() - tick1.toSec();
        if (duration > reconfiguration_duration) {
            current_headway = headway;
            ROS_WARN("current headway restore(quit mode):%f", current_headway);
        }
    }
}

double EMLike15::speed_control_1() {
    // dd=d_min; // 1.constant
    // dd=d_min+h_s*myselfcar.Velocity.v; // 2.+speed
    // dd=d_min+h_s*myselfcar.Velocity.v+G_s*std::pow(myselfcar.Velocity.v,2); // 3.+a

    double current_gap = ref_trajectory.r_s.back() - ref_trajectory.r_s[match_index];
    double pid_speed;
    double error;
    double dt = ros::Time::now().toSec() - kit1.last_time_.toSec();
    error = current_gap - current_headway;
    pid_speed = front_pos.speed + Kp * error;
    kit1.last_time_ = ros::Time::now();
    kit1.last_error_ = error;

    double euclidian_distance = sqrt(pow(front_pos.xPos - now_pos.xPos, 2) + pow(front_pos.yPos - now_pos.yPos, 2));
    std::cout << "current_headway : "<< current_headway << "(" << headway << ")" << ", current_gap : " << current_gap << ", error = " << error << "(euclidian distance = " << euclidian_distance << ")" << std::endl;
    std::cout << "front speed : "<< front_pos.speed << ", ego speed : " << now_pos.speed << ", raw pid_speed = " << pid_speed << std::endl;
    GapPlot(euclidian_distance);

    // //data record
    // DataRecordTool(current_gap);
    // DataRecordTool(current_headway);

    // limiter 1 : can not speed up or down too fast
    if (pid_speed < kit1.last_speed_ - max_deceleration) {
        pid_speed = kit1.last_speed_ - max_deceleration;
    } else if (pid_speed > kit1.last_speed_ + max_acceleration) {
        pid_speed = kit1.last_speed_ + max_acceleration;
    }
    // limiter 2 : can not back
    if (pid_speed < vmin) {
        pid_speed = vmin;
    } else if (pid_speed > 2 * vmax) {
        pid_speed = 2 * vmax;
    }

    kit1.last_speed_ = pid_speed;
    return pid_speed;
}

double EMLike15::speed_control_1(int traverse_i) {
    // dd=d_min; // 1.constant
    // dd=d_min+h_s*myselfcar.Velocity.v; // 2.+speed
    // dd=d_min+h_s*myselfcar.Velocity.v+G_s*std::pow(myselfcar.Velocity.v,2); // 3.+a

    double current_gap = ref_trajectory.r_s.back() - ref_trajectory.r_s[traverse_i];
    double pid_speed;
    double error;

    // double dt = ros::Time::now().toSec() - kit1.last_time_.toSec();
    error = current_gap - current_headway;
    pid_speed = front_pos.speed + Kp * error;
    // kit1.last_time_ = ros::Time::now();
    // kit1.last_error_ = error;

    // double euclidian_distance = sqrt(pow(front_pos.xPos - now_pos.xPos, 2) + pow(front_pos.yPos - now_pos.yPos, 2));
    // std::cout << "current_headway : "<< current_headway << "(" << headway << ")" << ", current_gap : " << current_gap << ", error = " << error << "(euclidian distance = " << euclidian_distance << ")" << std::endl;
    // std::cout << "front speed : "<< front_pos.speed << ", ego speed : " << now_pos.speed << ", raw pid_speed = " << pid_speed << std::endl;
    // GapPlot(euclidian_distance);

    // // limiter 1 : can not speed up or down too fast
    // if (pid_speed < kit1.last_speed_ - max_deceleration) {
    //     pid_speed = kit1.last_speed_ - max_deceleration;
    // } else if (pid_speed > kit1.last_speed_ + max_acceleration) {
    //     pid_speed = kit1.last_speed_ + max_acceleration;
    // }
    // limiter 2 : can not back
    if (pid_speed < vmin) {
        pid_speed = vmin;
    } else if (pid_speed > 2 * vmax) {
        pid_speed = 2 * vmax;
    }

    // kit1.last_speed_ = pid_speed;
    return pid_speed;
}

double EMLike15::calc_speed(int mode) {
    return speed_control_1();
}

std::array<double,2> EMLike15::forecast_position() {
    std::array<double,2> position;
    position[0] = now_pos.xPos + (now_pos.speed) * forecast_time * cos(now_pos.yaw);
    position[1] = now_pos.yPos + (now_pos.speed) * forecast_time * sin(now_pos.yaw);
    return position;
}

int EMLike15::findMatchpoint(double x, double y){
    int num = ref_trajectory.r_curvature.size();
    double dis_min = std::numeric_limits<double>::max();
    int index = 0;
    for (int i = 0; i < num; ++i) {
        double temp_dis = std::pow(ref_trajectory.r_x[i] - x, 2) + std::pow(ref_trajectory.r_y[i] - y, 2);
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

std::array<double,5> EMLike15::cal_err_k(){
    std::array<double,5> err_k;
    int target_index = match_index;

    //forecast, if small it will not work, if big it will overshoot
    std::array<double,2> forecast_pos = forecast_position();
    int forecast_index = findMatchpoint(forecast_pos[0], forecast_pos[1]);
    target_index = forecast_index;

    // direction vector
    Eigen::Matrix<double,2,1> tor, nor;
    tor << cos(ref_trajectory.r_yaw[target_index]), sin(ref_trajectory.r_yaw[target_index]);
    nor << -sin(ref_trajectory.r_yaw[target_index]), cos(ref_trajectory.r_yaw[target_index]);

    // cartesian error
    Eigen::Matrix<double,2,1> d_err;
    d_err << now_pos.xPos - ref_trajectory.r_x[target_index], now_pos.yPos - ref_trajectory.r_y[target_index];
    double phi = now_pos.yaw;
    double ed = nor.transpose() * d_err;
    double es = tor.transpose() * d_err;
    double projection_point_theta = ref_trajectory.r_yaw[target_index] + ref_trajectory.r_curvature[target_index] * es;
    double ed_d = vy * std::cos(phi - projection_point_theta) + vx * std::sin(phi - projection_point_theta);
    double ephi = phi - projection_point_theta;
    double s_d = (vx * std::cos(phi - projection_point_theta) -  vy * std::sin(phi - projection_point_theta)) /  (1 - ref_trajectory.r_curvature[target_index] * ed);
    double phi_d = vx * ref_trajectory.r_curvature[target_index];
    double ephi_d = phi_d - ref_trajectory.r_curvature[target_index] * s_d;
    err_k[0] = ed;
    err_k[1] = ed_d;
    err_k[2] = ephi;
    err_k[3] = ephi_d;
    err_k[4] = ref_trajectory.r_curvature[target_index];
    if(isnan(err_k[0])){
        ROS_WARN("ed error!!!");
        while(1);        
    }
    if(isnan(err_k[1])){
        ROS_WARN("ed_d error!!!");
        while(1);        
    }
    if(isnan(err_k[2])){
        ROS_WARN("ephi error!!!");
        while(1);        
    }
    if(isnan(err_k[3])){
        ROS_WARN("ephi_d error!!!");
        while(1);        
    }

    return err_k;
}

std::array<double,5> EMLike15::cal_err_k(double vx15, int traverse_i){
    std::array<double,5> err_k;
    int target_index = traverse_i;

    // //forecast, if small it will not work, if big it will overshoot
    // std::array<double,2> forecast_pos = forecast_position();
    // int forecast_index = findMatchpoint(forecast_pos[0], forecast_pos[1]);
    // target_index = forecast_index;

    // direction vector
    Eigen::Matrix<double,2,1> tor, nor;
    tor << cos(ref_trajectory.r_yaw[target_index]), sin(ref_trajectory.r_yaw[target_index]);
    nor << -sin(ref_trajectory.r_yaw[target_index]), cos(ref_trajectory.r_yaw[target_index]);

    // cartesian error
    Eigen::Matrix<double,2,1> d_err;
    d_err << 0, 0;
    double phi = ref_trajectory.r_yaw[target_index];
    double ed = nor.transpose() * d_err;
    double es = tor.transpose() * d_err;
    double projection_point_theta = ref_trajectory.r_yaw[target_index] + ref_trajectory.r_curvature[target_index] * es;
    double ed_d = vy * std::cos(phi - projection_point_theta) + vx * std::sin(phi - projection_point_theta);
    double ephi = phi - projection_point_theta;
    double s_d = (vx * std::cos(phi - projection_point_theta) -  vy * std::sin(phi - projection_point_theta)) /  (1 - ref_trajectory.r_curvature[target_index] * ed);
    double phi_d = vx * ref_trajectory.r_curvature[target_index];
    double ephi_d = phi_d - ref_trajectory.r_curvature[target_index] * s_d;
    err_k[0] = ed;
    err_k[1] = ed_d;
    err_k[2] = ephi;
    err_k[3] = ephi_d;
    err_k[4] = ref_trajectory.r_curvature[target_index];
    if(isnan(err_k[0])){
        ROS_WARN("err0 error!!!");
        while(1);        
    }
    if(isnan(err_k[1])){
        ROS_WARN("err1 error!!!");
        while(1);        
    }
    if(isnan(err_k[2])){
        ROS_WARN("err2 error!!!");
        while(1);        
    }
    if(isnan(err_k[3])){
        ROS_WARN("err3 error!!!");
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
bool EMLike15::cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double,4,1> B, Eigen::Matrix4d Q, Eigen::Matrix<double,1,1> R, Eigen::Matrix<double,1,4>& output){
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
        ROS_WARN("p0 error!!!");
        return false;   
    }
    if(isnan(p_old(1))){
        ROS_WARN("p1 error!!!");
        return false;        
    }
    if(isnan(p_old(2))){
        ROS_WARN("p2 error!!!");
        return false;       
    }
    if(isnan(p_old(3))){
        ROS_WARN("p3 error!!!");
        return false;        
    }
    output = (R + Bd.transpose() * p_old * Bd).inverse() * Bd.transpose() * p_old * Ad;
    if(isnan(output[0])){
        ROS_WARN("k0 error!!!");
        return false;       
    }
    if(isnan(output[1])){
        ROS_WARN("k1 error!!!");
        return false;       
    }
    if(isnan(output[2])){
        ROS_WARN("k2 error!!!");
        return false;         
    }
    if(isnan(output[3])){
        ROS_WARN("k3 error!!!");
        return false;        
    }
    return true;
}

bool EMLike15::cal_k(std::array<double,5> err_k, Eigen::Matrix<double,1,4>& output){
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
bool EMLike15::cal_k(std::array<double,5> err_k, Eigen::Matrix<double,1,4>& output, double vx15, int traverse_i){
    Eigen::Matrix4d A;
    Eigen::Matrix<double,4,1> B;
    Eigen::Matrix4d Q;
    Eigen::Matrix<double,1,1> R;
    A<<   0,            1,                                              0,                                0,
            0,           (cf+cr)/(m*vx15),                  -(cf+cr)/m,                (a*cf-b*cr)/(m*vx15),
            0,            0,                                              0,                                1,
            0,           (a*cf-b*cr)/(Iz*vx15),           -(a*cf-b*cr)/Iz,         (a*a*cf+b*b*cr)/(Iz*vx15);
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

double EMLike15::cal_forward_angle(Eigen::Matrix<double,1,4> k,std::array<double,5> err_k){
    double k3 = k[2];
    // 不足转向系数
    double kv = b * m / (cf * wheel_base) - a * m / (cr * wheel_base);
    double point_curvature = err_k[4];
    double forword_angle =  wheel_base * point_curvature + kv * vx * vx * point_curvature - k3 * (b * point_curvature - a * m * vx * vx * point_curvature / cr / b);
    if(isnan(forword_angle)){
        ROS_WARN("forword_angle error!!!");
        while(1);        
    }
    return forword_angle;
}
double EMLike15::cal_forward_angle(Eigen::Matrix<double,1,4> k,std::array<double,5> err_k, double vx15, int traverse_i){
    double k3 = k[2];
    // 不足转向系数
    double kv = b * m / (cf * wheel_base) - a * m / (cr * wheel_base);
    double point_curvature = err_k[4];
    double forword_angle =  wheel_base * point_curvature + kv * vx15 * vx15 * point_curvature - k3 * (b * point_curvature - a * m * vx15 * vx15 * point_curvature / cr / b);
    if(isnan(forword_angle)){
        ROS_WARN("forword_angle error!!!");
        while(1);        
    }
    return forword_angle;
}

double EMLike15::cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle, std::array<double, 5> err_k){
    Eigen::Matrix<double, 4, 1> err;
    err << err_k[0], err_k[1], err_k[2], err_k[3];
    double angle = -k * err + forword_angle;
    if(isnan(angle)){
        ROS_WARN("angle error!!!");
        while(1);        
    }
    return angle;
}

double EMLike15::theat_angle(){
    // error state space model
    std::array<double,5> err_k = cal_err_k();
    std::cout << "[LQR] ed=" << err_k[0] << ", ed_d=" << err_k[1] << ", ephi=" << err_k[2] << ", ephi_d" << err_k[3] << std::endl;
    // u = -kx + deltaf : k
    Eigen::Matrix<double,1,4> k;
    if (!cal_k(err_k, k)) {
        return 0.0;
    }
    std::cout << "[emlike15]: k= " << k << std::endl;
    // u = -kx + deltaf : deltaf
    double forword_angle = cal_forward_angle(k,err_k);
    std::cout << "[emlike15]: feedforward= " << forword_angle << std::endl;
    // u = -kx + deltaf
    double lqr_delta = cal_angle(k, forword_angle, err_k);
    std::cout << "[emlike15]: raw_lqr_delta= " << lqr_delta << std::endl;
    if (lqr_delta > wmax) {
        lqr_delta = wmax;
    } else if (lqr_delta < wmin) {
        lqr_delta = wmin;
    }

    return lqr_delta;
}

double EMLike15::theat_angle(double vx15, int traverse_i){
    // error state space model
    std::array<double,5> err_k = cal_err_k(vx15, traverse_i);
    std::cout << "[emlike15]: error: " << err_k[0] << "," << err_k[1] << "," << err_k[2] << "," << err_k[3] << "," << err_k[4] << "," << std::endl;
    // u = -kx + deltaf : k
    Eigen::Matrix<double,1,4> k;
    if (!cal_k(err_k, k)) {
        return 0.0;
    }
    std::cout << "[emlike15]: k= " << k << std::endl;
    // u = -kx + deltaf : deltaf
    double forword_angle = cal_forward_angle(k,err_k, vx15, traverse_i);
    std::cout << "[emlike15]: feedforward= " << forword_angle << std::endl;
    // u = -kx + deltaf
    double lqr_delta = cal_angle(k, forword_angle, err_k);
    std::cout << "[emlike15]: raw_lqr_delta= " << lqr_delta << std::endl;
    if (lqr_delta > wmax) {
        lqr_delta = wmax;
    } else if (lqr_delta < wmin) {
        lqr_delta = wmin;
    }

    return lqr_delta;
}

void EMLike15::EMLike15Core(){
    std::cout << "\033[32m---emlike15 entered---\033[0m" << std::endl;

    if(ref_trajectory.r_s.empty()){
        ROS_WARN("[control] : NO trajectory of spline");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_pub.publish(cmd_vel);
        return;
    }

    // if (GetVehicleInfo(0, leader_pos)) {
    //     // return;
    // }
    // if (GetVehicleInfo(ID - 1, front_pos)) {
    //     // return;
    //     printf("front info : x = %f, y = %f\n", front_pos.xPos, front_pos.yPos);
    // }
    LoadCustomLoc(fmodom[target_ID], front_pos);

    // quit mode modify headway
    if (ID == 2) {
        // std::cout << "modify headway\n";
        ModifyHeadway();
    }

    match_index = findMatchpoint(now_pos.xPos, now_pos.yPos);
    std::cout << "[emlike15]: match_index:" << match_index << std::endl;

    double speed_final = calc_speed(1);
    double delta_final;
    double aspeed_final;
    if (vx < 1e-4) {
        aspeed_final = 0.0;
    } else {
        delta_final = theat_angle();

        // ackerman to differential
        aspeed_final = speed_final * std::tan(delta_final) / wheel_base;
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = speed_final;
    cmd_vel.angular.z = aspeed_final;
    cmd_pub.publish(cmd_vel);
    std::cout << "[emlike15] : v = " << speed_final << ", w = " << aspeed_final << std::endl;

    // // 15base need tmp to note
    custom_messages::track15 formation_trajectory;
    formation_trajectory.Header.stamp = ros::Time::now();
    std::vector<double> speed;
    for (int i = match_index; i < ref_trajectory.r_s.size(); ++i) {
        geometry_msgs::Point tmp_point;
        tmp_point.x = ref_trajectory.r_x[i];
        tmp_point.y = ref_trajectory.r_y[i];
        formation_trajectory.position.emplace_back(tmp_point);
        double speed_tmp = speed_control_1(i);
        formation_trajectory.v.emplace_back(speed_tmp);
        speed.emplace_back(speed_tmp);
        formation_trajectory.yaw.emplace_back(ref_trajectory.r_yaw[i]);
        formation_trajectory.curvature.emplace_back(ref_trajectory.r_curvature[i]);
    }
    std::vector<double> acc;
    acc.resize(speed.size());
    for (int i = 0; i < speed.size(); ++i) {
        if (i + 1 < speed.size()) {
            acc[i] = (speed[i + 1] - speed[i]) * emlike15_hz;
        } else {
            acc[i] = acc[i - 1];
        }
    }
    for (int i = match_index; i < ref_trajectory.r_s.size(); ++i) {
        formation_trajectory.acc.emplace_back(acc[i - match_index]);
    }
    full_trajectory_pub.publish(formation_trajectory);

    if (reconfiguration_flag != 0) {
        last_reconfiguration_flag = reconfiguration_flag;
    }

    std_msgs::Float64 ros_followdistance;
    ros_followdistance.data = current_headway;
    C_followdistance_pub.publish(ros_followdistance);

    fixpb4performance();
}

void EMLike15::fixpb4performance() {
    fixpb4lateral_error();
    fixpb4yaw_error();
    fixpb4follow_error();
}

void EMLike15::fixpb4lateral_error() {
    std_msgs::Float64 tmp_lateral_error;
    if (abs(fmodom[target_ID].yaw - M_PI / 2) < 1e-6 || abs(fmodom[target_ID].yaw + M_PI / 2) < 1e-6) {
        double lateral_error = abs(fmodom[target_ID].xPos - fmodom[ID].xPos);
        tmp_lateral_error.data = lateral_error;
    } else {
        double k = tan(fmodom[target_ID].yaw);
        double A = k;
        double B = -1.0;
        double C = fmodom[target_ID].yPos - k * fmodom[target_ID].xPos;
        double x = fmodom[ID].xPos;
        double y = fmodom[ID].yPos;
        double lateral_d = sqrt(pow((A * x + B * y + C), 2)) / sqrt(pow(A, 2) + pow(B, 2));
        tmp_lateral_error.data = lateral_d;
    }
    lateral_error_pub.publish(tmp_lateral_error);
}

void EMLike15::fixpb4yaw_error() {
    std_msgs::Float64 tmp_yaw_error;
    double yaw_error = abs(fmodom[target_ID].yaw - fmodom[ID].yaw);
    tmp_yaw_error.data = yaw_error;
    yaw_error_pub.publish(tmp_yaw_error);
}

void EMLike15::fixpb4follow_error() {
    std_msgs::Float64 tmp_follow_error;
    double follow_error = ref_trajectory.r_s.back() - ref_trajectory.r_s[match_index] - current_headway;
    tmp_follow_error.data = follow_error;
    follow_error_pub.publish(tmp_follow_error);
}

const std::string EMLike15::DataRecord() {
    time_t currentTime = time(NULL);
    char chCurrentTime[64];
    strftime(chCurrentTime, sizeof(chCurrentTime), "%Y%m%d-%H%M%S", localtime(&currentTime));  //年月日 时分秒
    std::string stCurrentTime = chCurrentTime;
    //
    std::string file_Dir = "./path_log";  // 新建存储文件夹
    if (-1 == access(file_Dir.c_str(), F_OK)) {
        mkdir(file_Dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    }
    //
    std::string myFileName = " ";
    myFileName = file_Dir + "/path_" + stCurrentTime + ".txt";//存储数据的文件名
    // open file
    if (!file_to_write_.is_open()) {
        file_to_write_.open(myFileName, std::ios::out);// 写 文件不存在则建立新文件 文件存在则清空文件内容
    }
    return myFileName;
}

void EMLike15::DataRecordTool(double data1) {
    file_to_write_ << std::fixed << std::setprecision(3) << std::right << std::setw(15) << data1 << " ";
}

int main(int argc, char  **argv)
{
    ros::init(argc,argv,"emlike15");
    ros::NodeHandle nh;

    int recv_ID, recv_target_ID;
    nh.param<int>("recv_ID", recv_ID, 1);
    nh.param<int>("recv_target_ID", recv_target_ID, 0);
    nh.param<double>("emlike15/emlike15_hz", emlike15_hz, 20.0);
    nh.param<double>("headway", headway, 4.0);
    current_headway = headway;
    nh.param<double>("vmin", vmin, -0.6);
    nh.param<double>("vmax", vmax, 1.2);
    nh.param<double>("wmin", wmin, -1.57);
    nh.param<double>("wmax", wmax, 1.57);
    nh.param<double>("overall_length", overall_length, 0.6);
    nh.param<double>("Kp", Kp, 0.9);
    nh.param<double>("Ki", Ki, 0.2);
    nh.param<double>("Kd", Kd, 1.0);
    nh.param<double>("max_deceleration", max_deceleration, 0.3);
    nh.param<double>("max_acceleration", max_acceleration, 0.3);
    nh.param<double>("wheel_base", wheel_base, 0.48);
    nh.param<double>("cf", cf, -115494663);
    nh.param<double>("cr", cr, -115494663);
    nh.param<double>("weight", m, 20.0);
    nh.param<double>("a", a, 0.24);
    nh.param<double>("b", b, 0.24);
    Iz = std::pow(a, 2) * m/2 + std::pow(b, 2) * m/2; // double Iz = std::pow(a, 2) * mass_front + std::pow(b, 2) * mass_rear;
    nh.param<double>("lqr_q1", Q1, 60.0);
    nh.param<double>("lqr_q2", Q2, 1.0);
    nh.param<double>("lqr_q3", Q3, 60.0);
    nh.param<double>("lqr_q4", Q4, 1.0);
    nh.param<double>("lqr_r", lqrR, 35.0);
    nh.param<double>("time_headway", time_headway, 0.6);
    // nh.param<int>("/formation_type", formation_type, 0);
    nh.param<double>("forecast_time", forecast_time, 0.2);
    nh.param<double>("reconfiguration_duration", reconfiguration_duration, 10.0);
    nh.param<double>("merge_lateral_threshold", merge_lateral_threshold, 1.5);
    nh.param<double>("merge_yaw_threshold", merge_yaw_threshold, M_PI / 12);

    ros::Publisher vis_trajectory_pub = nh.advertise<nav_msgs::Path>("VisTraj", 100);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    C_followdistance_pub = nh.advertise<std_msgs::Float64>("/C_followdistance", 1);
    full_trajectory_pub = nh.advertise<custom_messages::track15>("formation_trajectory", 1);

    lateral_error_pub = nh.advertise<std_msgs::Float64>("fixpb4lateral_error", 1);
    yaw_error_pub = nh.advertise<std_msgs::Float64>("fixpb4yaw_error", 1);
    follow_error_pub = nh.advertise<std_msgs::Float64>("fixpb4follow_error", 1);

    gap_plot_pub = nh.advertise<std_msgs::Float64>("gap_error", 20);
    current_gap_pub = nh.advertise<std_msgs::Float64>("gap", 20);
    ideal_gap_pub = nh.advertise<std_msgs::Float64>("gap_ideal", 20);

    EMLike15* myemlike15 = new EMLike15(vis_trajectory_pub, recv_ID, recv_target_ID);

    // ros::Subscriber reconfiguration_sub = nh.subscribe("/reID", 1, ReconfigurationCallback);

    ros::Subscriber A_sub = nh.subscribe("/number1/odom15", 10, &EMLike15::ACallback, myemlike15);
    ros::Subscriber B_sub = nh.subscribe("/number2/odom15", 10, &EMLike15::BCallback, myemlike15);
    ros::Subscriber C_sub = nh.subscribe("/number3/odom15", 10, &EMLike15::CCallback, myemlike15);
    ros::Subscriber cs15_sub = nh.subscribe("trajectory_cs15", 10, &EMLike15::refTrajCallback, myemlike15);

    max_deceleration /= emlike15_hz;
    max_acceleration /= emlike15_hz;
    
    ros::Rate loop_rate(emlike15_hz);
    while (ros::ok()) {
        myemlike15->reconfiguration_flag = 0;

        ros::spinOnce();
    
        std::cout << "---emlike15 prework start---" << std::endl;
        std::cout << "[NAME] EGO:number" << myemlike15->ID + 1 << ", " << "TARGET:number" << myemlike15->target_ID + 1 << std::endl;
        myemlike15->EMLike15Prework();
        std::cout << "---emlike15 prework done---" << std::endl;
        // ros::Time time1 = ros::Time::now();
        std::cout << "---emlike15 core start---" << std::endl;
        myemlike15->EMLike15Core();
        std::cout << "---emlike15 core done---" << std::endl;
        // ros::Time time2 = ros::Time::now();
        // std::printf("[em_like] : Time consume is : %f\n",(time2 - time1).toSec() );

        loop_rate.sleep();
    }
    return 0;
}