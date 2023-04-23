#include "monitor.h"

double monitor_hz;
double lateral_tolerance;
double yaw_tolerance;
double longitudinal_tolerance;

void Monitor::Platoon0Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    mList[0].first = 1;
    mList[0].second = *p_msg;
}
void Monitor::Platoon1Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    mList[1].first = 1;
    mList[1].second = *p_msg;
}
void Monitor::Platoon2Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    mList[2].first = 1;
    mList[2].second = *p_msg;
}
void Monitor::Platoon3Callback(const nav_msgs::Odometry::ConstPtr& p_msg){
    mList[3].first = 1;
    mList[3].second = *p_msg;
}

Monitor::Monitor(ros::Publisher& start_check_pub){
    _start_check_pub = start_check_pub;
    mList.resize(10);
}

Monitor::Monitor() {
    mList.resize(10);
}

bool Monitor::GetVehicleInfo(int index, custom_messages::vehicle_status& out) {
    if (mList[index].first == 0) {
        // printf("[em_like]: get %i info failed\n", index);
        return false;
    }
    nav_msgs::Odometry tmp(mList[index].second);
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

bool Monitor::start_check() {
    double lateral_error, yaw_error, longitudinal_error;
    custom_messages::vehicle_status leader;
    GetVehicleInfo(0, leader);

    // lateral and longitudinal error check
    if (abs(leader.yaw - M_PI / 2) < 1e-6 || abs(leader.yaw + M_PI / 2) < 1e-6) {
        for (int i = 1; i < mList.size(); ++i) {
            if (mList[i].first == 1) {
                lateral_error = abs(mList[i].second.pose.pose.position.x - leader.xPos);
                if (lateral_error > lateral_tolerance) {
                    ROS_ERROR("unit[%i] lateral error", i);
                    return false;
                }

                longitudinal_error = abs(mList[i].second.pose.pose.position.y - leader.yPos);
                if (longitudinal_error > longitudinal_tolerance) {
                    ROS_ERROR("unit[%i] longitudinal error", i);
                    return false;
                }
            }
        }
    } else {
        double k = tan(leader.yaw);
        double A = k;
        double B = -1.0;
        double C = leader.yPos - k * leader.xPos;
        for (int i = 1; i < mList.size(); ++i) {
            if (mList[i].first == 1) {
                double x = mList[i].second.pose.pose.position.x;
                double y = mList[i].second.pose.pose.position.y;
                double lateral_d = sqrt(pow((A * x + B * y + C), 2)) / sqrt(pow(A, 2) + pow(B, 2));
                if (lateral_d > lateral_tolerance) {
                    ROS_ERROR("unit[%i] lateral error(%f)", i, lateral_d);
                    return false;
                }
                double d = sqrt(pow(x - leader.xPos, 2) + pow(y - leader.yPos, 2));
                double longitudinal_d = sqrt(d * d - lateral_d * lateral_d);
                if (longitudinal_d > longitudinal_tolerance) {
                    ROS_ERROR("unit[%i] longitudinal error", i);
                    return false;
                }
            }
        }
    }
    // yaw error check
    for (int i = 1; i < mList.size(); ++i) {
        custom_messages::vehicle_status tmp;
        if (!GetVehicleInfo(i, tmp)) {
            continue;
        } else {
            if (abs(tmp.yaw - leader.yaw) > yaw_tolerance) {
                ROS_ERROR("unit[%i] yaw error", i);
                return false;
            }
        }
    }
    return true;
}

bool Monitor::online_check() {
    int count = 0;
    std::vector<int> count_name;
    for (int i = 0; i < mList.size(); ++i) {
        if (mList[i].first == 1) {
            ++count;
            count_name.emplace_back(i);
        }
    }
    ROS_INFO("[online count] total : 3, current : %i", count);
    if (count != 3) {
        ROS_ERROR("online unit MISSING!!!");
        return false;
    }
    if (count != 0) {
        std::cout << "online units are: ";
        for (auto x : count_name) {
            std::cout << "No_" << x << " ";
        }
        std::cout << std::endl;
    }
    return true;
}

void Monitor::run() {
    std::cout << std::endl;
    std::cout << "----MONITOR LOG----" << std::endl;

    // online check
    if (!online_check()) {
        return;
    }

    // start_check
    if (start_check()) {
        ROS_INFO("[start check]: Platoon is ready to go");
    } else {
        ROS_ERROR("[start check]: Platoon can NOT launch!!!");
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "monitor");
    ros::NodeHandle n;

    // n.param<int>("ID", ID, 0);
    n.param<double>("monitor/lateral_tolerance", lateral_tolerance, 0.8);
    n.param<double>("monitor/yaw_tolerance", yaw_tolerance, M_PI / 6);
    n.param<double>("monitor/longitudinal_tolerance", longitudinal_tolerance, 1.4);
    n.param<double>("monitor/monitor_hz", monitor_hz, 1.0);

    // ros::Publisher start_check_pub = n.advertise<std_msgs::String>("start_check", 1);

    // Monitor* myMonitor = new Monitor(start_check_pub);
    Monitor* myMonitor = new Monitor();

    ros::Subscriber platoon0_sub = n.subscribe("/No_0/myodom", 5, &Monitor::Platoon0Callback, myMonitor);
    ros::Subscriber platoon1_sub = n.subscribe("/No_1/myodom", 5, &Monitor::Platoon1Callback, myMonitor);
    ros::Subscriber platoon2_sub = n.subscribe("/No_2/myodom", 5, &Monitor::Platoon2Callback, myMonitor);
    ros::Subscriber platoon3_sub = n.subscribe("/No_3/myodom", 5, &Monitor::Platoon3Callback, myMonitor);

    ros::Rate rate(monitor_hz);
    while(ros::ok()){
        ros::spinOnce();

        myMonitor->run();

        rate.sleep();
    }
}