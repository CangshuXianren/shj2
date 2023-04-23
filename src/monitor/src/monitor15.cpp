#include "monitor/monitor15.h"

ros::Publisher start_check_pub;

double monitor_hz;
double lateral_tolerance;
double yaw_tolerance;
double longitudinal_tolerance;

double set_headway;

int sock;
sockaddr_in saddr;

void Monitor15::ACallback(const custom_messages::status15::ConstPtr& p_msg){
    // std::cout << "[cs15]: get A loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    if (p_msg != nullptr) {
        mList[0].first = true;
        mList[0].second = *p_msg;
    }
}
void Monitor15::BCallback(const custom_messages::status15::ConstPtr& p_msg){
    // std::cout << "[cs15]: get B loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    if (p_msg != nullptr) {
        mList[1].first = true;
        mList[1].second = *p_msg;
    }
}
void Monitor15::CCallback(const custom_messages::status15::ConstPtr& p_msg){
    // std::cout << "[cs15]: get C loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    if (p_msg != nullptr) {
        mList[2].first = true;
        mList[2].second = *p_msg;
    }
}

void Monitor15::CTargetCallback(const std_msgs::Int32::ConstPtr& p_msg) {
    C_target_ = p_msg->data;
    get_Ctarget_flag = true;
    // std::cout << "c_target: " << C_target_ << std::endl;
}

void Monitor15::CFollowdistanceCallback(const std_msgs::Float64::ConstPtr& p_msg) {
    C_followdistance_ = p_msg->data;
    get_Cfollowdistance_flag = true;
}

void Monitor15::Status15toCustomLoc(custom_messages::status15& input, custom_messages::vehicle_status& output) {
    output.xPos = input.position.x;
    output.yPos = input.position.y;
    output.yaw = input.pose.yaw;
    output.speed = input.linear_velocity.x;
    output.angular_speed = input.angular_velocity.z;
}

bool Monitor15::start_check() {
    double lateral_error, yaw_error, longitudinal_error;
    custom_messages::vehicle_status leader;
    Status15toCustomLoc(mList[0].second, leader);
    bool start_check_flag = true;

    // lateral and longitudinal error check
    if (abs(leader.yaw - M_PI / 2) < 1e-6 || abs(leader.yaw + M_PI / 2) < 1e-6) {
        for (int i = 1; i < mList.size(); ++i) {
            lateral_error = abs(mList[i].second.position.x - leader.xPos);
            if (lateral_error > lateral_tolerance) {
                std::cout << RED << "unit[" << i << "] lateral error(" << lateral_error << ")" << RESET << std::endl;
                start_check_flag = false;
            }
            longitudinal_error = abs(mList[i].second.position.y - leader.yPos);
            if (longitudinal_error > longitudinal_tolerance) {
                std::cout << "RED unit[" << i << "] longitudinal error(" << longitudinal_error << ")" << RESET << std::endl;
                start_check_flag = false;
            }
        }
    } else {
        double k = tan(leader.yaw);
        double A = k;
        double B = -1.0;
        double C = leader.yPos - k * leader.xPos;
        for (int i = 1; i < mList.size(); ++i) {
            double x = mList[i].second.position.x;
            double y = mList[i].second.position.y;
            double lateral_d = sqrt(pow((A * x + B * y + C), 2)) / sqrt(pow(A, 2) + pow(B, 2));
            if (lateral_d > lateral_tolerance) {
                std::cout << RED << "unit[" << i << "] lateral error(" << lateral_d << ")" << RESET << std::endl;
                start_check_flag = false;
            }
            double d = sqrt(pow(x - leader.xPos, 2) + pow(y - leader.yPos, 2));
            double longitudinal_d = sqrt(d * d - lateral_d * lateral_d);
            if (longitudinal_d > longitudinal_tolerance) {
                std::cout << RED << "unit[" << i << "] longitudinal error(" << longitudinal_d << ")" << RESET << std::endl;
                start_check_flag = false;
            }
        }
    }
    // yaw error check
    for (int i = 1; i < mList.size(); ++i) {
        custom_messages::vehicle_status tmp;
        Status15toCustomLoc(mList[i].second, tmp);
        if (abs(tmp.yaw - leader.yaw) > yaw_tolerance) {
            std::cout << RED << "unit[" << i << "] yaw error(" << abs(tmp.yaw - leader.yaw) << ")" << RESET << std::endl;
            start_check_flag = false;
        } 
    }
    return start_check_flag;
}

void Monitor15::run() {
    std::cout << std::endl;
    std::cout << GREEN << "----MONITOR LOG----" << RESET << std::endl;

    if (!online_check()) {
        // return;
    }

    if (!get_Ctarget_flag) {
        std::cout << RED << "C target not recv" << RESET << std::endl;
        return;
    }
    if (!get_Cfollowdistance_flag) {
        std::cout << RED << "C follow distance not recv" << RESET << std::endl;;
        return;
    }
    std_msgs::Bool flag;
    if (start_check_button_) {
        if (start_check()) {
            ROS_INFO("[start check]: Platoon is ready to go");
            flag.data = true;
            start_check_pub.publish(flag);
        } else {
            ROS_ERROR("[start check]: Platoon can NOT launch!!!");
            flag.data = false;
            start_check_pub.publish(flag);
        }
    }

    BroadcastChiefcmd();
}

void Monitor15::BroadcastChiefcmd() {
    fm::FormationMsg chief;
    chief.set_msg_type_(fm::Msg_Type::Chief);
    fm::Chief_cmd* chief_cmd = chief.mutable_chief_cmd_();
    chief_cmd->set_name_("number1");
    chief_cmd->set_time_(ros::Time::now().toSec());
    chief_cmd->set_b_formatio_state_(mList[1].second.self_state);
    chief_cmd->set_c_formatio_state_(mList[2].second.self_state);

    fm::follow_information* b_follow = chief_cmd->mutable_b_follow_information_();
    fm::target_information* b_target = b_follow->mutable_target_information_();
    fm::position* b_target_position = b_target->mutable_position_();
    b_target_position->set_x(mList[0].second.position.x);
    b_target_position->set_y(mList[0].second.position.y);
    fm::pose* b_target_pose = b_target->mutable_pose_();
    b_target_pose->set_yaw(mList[0].second.pose.yaw);
    b_follow->set_follow_distance_(set_headway);

    fm::follow_information* c_follow = chief_cmd->mutable_c_follow_information_();
    fm::target_information* c_target = c_follow->mutable_target_information_();
    fm::position* c_target_position = c_target->mutable_position_();
    c_target_position->set_x(mList[C_target_].second.position.x);
    c_target_position->set_y(mList[C_target_].second.position.y);
    fm::pose* c_target_pose = c_target->mutable_pose_();
    c_target_pose->set_yaw(mList[C_target_].second.pose.yaw);
    c_follow->set_follow_distance_(C_followdistance_);

    int len = chief.ByteSize();
    char *buff = new char[len + 1];
    chief.SerializeToArray(buff, len);
    int len_ = sendto(sock, buff, len, 0, (sockaddr*)&saddr, sizeof(saddr));

    std::cout << "b_formation_state: " << mList[1].second.self_state << std::endl;
    std::cout << "c_formation_state: " << mList[2].second.self_state << std::endl;
    std::cout << "c_target: " << C_target_ << std::endl;
    std::cout << "c_follow_distance: " << C_followdistance_ << std::endl;
}

void Monitor15::BroadcastChiefcmdFakeTest() {
    ROS_INFO("start to send test msg");
    fm::FormationMsg chief;
    chief.set_msg_type_(fm::Msg_Type::Chief);
    fm::Chief_cmd* chief_cmd = chief.mutable_chief_cmd_();
    chief_cmd->set_name_("number1");
    chief_cmd->set_time_(ros::Time::now().toSec());
    chief_cmd->set_b_formatio_state_(0);
    chief_cmd->set_c_formatio_state_(0);

    fm::follow_information* b_follow = chief_cmd->mutable_b_follow_information_();
    fm::target_information* b_target = b_follow->mutable_target_information_();
    fm::position* b_target_position = b_target->mutable_position_();
    b_target_position->set_x(100.0);
    b_target_position->set_y(200.0);
    fm::pose* b_target_pose = b_target->mutable_pose_();
    b_target_pose->set_yaw(300.0);
    b_follow->set_follow_distance_(3.0);

    fm::follow_information* c_follow = chief_cmd->mutable_c_follow_information_();
    fm::target_information* c_target = c_follow->mutable_target_information_();
    fm::position* c_target_position = c_target->mutable_position_();
    c_target_position->set_x(400.0);
    c_target_position->set_y(500.0);
    fm::pose* c_target_pose = c_target->mutable_pose_();
    c_target_pose->set_yaw(600.0);
    c_follow->set_follow_distance_(4.0);

    int len = chief.ByteSize();
    ROS_INFO("bytesize:%i", len);
    char *buff = new char[len + 1];
    chief.SerializeToArray(buff, len);
    int len_ = sendto(sock, buff, len, 0, (sockaddr*)&saddr, sizeof(saddr));
    std::cout << "sendto len_ is: " << len_ << std::endl;
}

bool Monitor15::online_check() {
    std::cout << "--online_check start--" << std::endl;
    int count = 0;
    std::vector<int> count_name;
    for (int i = 0; i < mList.size(); ++i) {
        if (mList[i].first == true) {
            ++count;
            count_name.emplace_back(i);
        }
    }
    ROS_INFO("[online count] total : 3, current : %i", count);
    if (count != 3) {
        ROS_ERROR("online unit MISSING!!!");
        std::cout << "online units are: ";
        for (auto x : count_name) {
            std::cout << "number" << x << " ";
        }
        std::cout << std::endl;
        std::cout << "--online_check done--" << std::endl;
        return false;
    }
    std::cout << "--online_check done--" << std::endl;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "monitor15");
    ros::NodeHandle n;

    int set_port;
    bool test_mode, start_check_button;
    n.param<double>("monitor15/lateral_tolerance", lateral_tolerance, 0.8);
    n.param<double>("monitor15/yaw_tolerance", yaw_tolerance, M_PI / 6);
    n.param<double>("monitor15/longitudinal_tolerance", longitudinal_tolerance, 1.4);
    n.param<double>("monitor15/monitor_hz", monitor_hz, 1.0);
    n.param<int>("set_port", set_port, 8001);
    n.param<double>("headway", set_headway, 1.0);
    n.param<bool>("monitor15/test_mode", test_mode, false);
    n.param<bool>("monitor15/start_check_button", start_check_button, false);

    // udp
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&opt, sizeof(opt));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(set_port);
    saddr.sin_addr.s_addr = INADDR_BROADCAST;
    saddr.sin_addr.s_addr = inet_addr("192.168.1.50");

    start_check_pub = n.advertise<std_msgs::Bool>("/start_check", 1);

    Monitor15* myMonitor = new Monitor15();
    myMonitor->start_check_button_ = start_check_button;

    ros::Subscriber A_sub = n.subscribe("/number1/odom15", 5, &Monitor15::ACallback, myMonitor);
    ros::Subscriber B_sub = n.subscribe("/number2/odom15", 5, &Monitor15::BCallback, myMonitor);
    ros::Subscriber C_sub = n.subscribe("/number3/odom15", 5, &Monitor15::CCallback, myMonitor);

    ros::Subscriber C_target_sub = n.subscribe("/C_target", 5, &Monitor15::CTargetCallback, myMonitor);
    ros::Subscriber C_followdistance_sub = n.subscribe("/C_followdistance", 5, &Monitor15::CFollowdistanceCallback, myMonitor);

    ros::Rate rate(monitor_hz);
    while(ros::ok()){
        ros::spinOnce();

        if (test_mode) {
            myMonitor->BroadcastChiefcmdFakeTest();
        } else {
            myMonitor->run();
        }

        rate.sleep();
    }
}