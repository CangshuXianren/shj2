#include "bridge.h"

void Bridge::init(unsigned short& port, int& sock) {
    ROS_INFO("start init");
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    // int opt = 1;
    // setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&opt, sizeof(opt));
    sockaddr_in saddr;
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(port);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    // saddr.sin_addr.s_addr = inet_addr("192.168.1.102");
    if (bind(sock, (struct sockaddr*)&saddr, sizeof(struct sockaddr)) == -1) {
        perror("bind error\n");
        exit(1);
    }
    ROS_INFO("init done");
}

void Bridge::run(int& sock, char (&buffer)[1024]) {
    std::cout << std::endl;
    std::cout << "\033[32m---BRIDGE RUN ONCE---\033[0m" << std::endl;
    size_t recvnum = recvfrom(sock, buffer, sizeof(buffer), 0, 0, 0);
    std::cout << "recvnum: " << recvnum << std::endl;

    fm::FormationMsg pb_msg;
    pb_msg.ParseFromArray(buffer, recvnum);
    std::cout << "receive msg type is: " << pb_msg.msg_type_() << std::endl;

    if (pb_msg.msg_type_() == fm::Msg_Type::ODOM) {
        if (!pb_msg.has_odom_()) {
            std::cout << "odom pb empty!" << std::endl;
        }
        std::cout << "name: " << pb_msg.odom_().name_() << std::endl;
        custom_messages::status15 odom_tmp;
        nav_msgs::Odometry rosodom_tmp;
        rosodom_tmp.header.frame_id = "map";
        fm2status15(pb_msg, odom_tmp);
        if (pb_msg.odom_().name_() == "number1") {
            number1_pub.publish(odom_tmp);
            Status15toRosodometry(odom_tmp, rosodom_tmp);
            number1_vis_pub.publish(rosodom_tmp);
        } else if (pb_msg.odom_().name_() == "number2") {
            number2_pub.publish(odom_tmp);
            Status15toRosodometry(odom_tmp, rosodom_tmp);
            number2_vis_pub.publish(rosodom_tmp);
        } else if (pb_msg.odom_().name_() == "number3") {
            number3_pub.publish(odom_tmp);
            Status15toRosodometry(odom_tmp, rosodom_tmp);

            geometry_msgs::Quaternion q;
            q = tf::createQuaternionMsgFromRollPitchYaw(odom_tmp.pose.roll, odom_tmp.pose.pitch, odom_tmp.pose.yaw);
            rosodom_tmp.pose.pose.orientation.w = q.w;
            rosodom_tmp.pose.pose.orientation.x = q.x;
            rosodom_tmp.pose.pose.orientation.y = q.y;
            rosodom_tmp.pose.pose.orientation.z = q.z;

            number3_vis_pub.publish(rosodom_tmp);
        }
    } else if (pb_msg.msg_type_() == fm::Msg_Type::Chief) {
        if (!pb_msg.has_chief_cmd_()) {
            std::cout << "chief pb empty!" << std::endl;
        }
        if (log_mode_) {
            std::cout << "b formation state: " << pb_msg.chief_cmd_().b_formatio_state_();
            std::cout << "c target x: " << pb_msg.chief_cmd_().c_follow_information_().target_information_().position_().x();
        }
        custom_messages::chief_cmd chiefcmd_tmp;
        fm2chiefcmd(pb_msg, chiefcmd_tmp);
        chief_pub.publish(chiefcmd_tmp);
    }
}

void Bridge::Status15toRosodometry(custom_messages::status15& input, nav_msgs::Odometry& output) {
    output.pose.pose.position.x = input.position.x;
    output.pose.pose.position.y = input.position.y;
    output.pose.pose.orientation.w = input.orientation.w;
    output.pose.pose.orientation.x = input.orientation.x;
    output.pose.pose.orientation.y = input.orientation.y;
    output.pose.pose.orientation.z = input.orientation.z;
}

void Bridge::fm2status15(fm::FormationMsg& input, custom_messages::status15& output) {
    output.name = input.odom_().name_();
    output.time = input.odom_().time_();
    output.self_state = input.odom_().self_state_();
    output.request = input.odom_().request_();
    output.gps_position.altitude = input.odom_().gps_position_().altitude();
    output.gps_position.latitude = input.odom_().gps_position_().latitude();
    output.gps_position.longitude = input.odom_().gps_position_().longitude();
    output.position.x = input.odom_().position_().x();
    output.position.y = input.odom_().position_().y();
    output.position.z = input.odom_().position_().z();
    output.pose.pitch = input.odom_().pose_().pitch();
    output.pose.roll = input.odom_().pose_().roll();
    output.pose.yaw = input.odom_().pose_().yaw();
    output.orientation.w = input.odom_().orientation_().w();
    output.orientation.x = input.odom_().orientation_().x();
    output.orientation.y = input.odom_().orientation_().y();
    output.orientation.z = input.odom_().orientation_().z();
    output.linear_velocity.x = input.odom_().linear_velocity_().x();
    output.linear_velocity.y = input.odom_().linear_velocity_().y();
    output.linear_velocity.z = input.odom_().linear_velocity_().z();
    output.angular_velocity.x = input.odom_().angular_velocity_().x();
    output.angular_velocity.y = input.odom_().angular_velocity_().y();
    output.angular_velocity.z = input.odom_().angular_velocity_().z();
    output.performance.follow_distance_error = input.odom_().performance_().follow_distance_error();
    output.performance.lateral_position_error = input.odom_().performance_().lateral_position_error();
    output.performance.yaw_angular_error = input.odom_().performance_().yaw_angular_error();
    if (log_mode_) {
        std::cout << "[name]" << output.name << ":" << std::endl;
        std::cout << "x: " << output.position.x << std::endl;
        std::cout << "y: " << output.position.y << std::endl;
        std::cout << "yaw: " << output.pose.yaw << std::endl;
        std::cout << "speed: " << output.linear_velocity.x << std::endl;
        std::cout << "w:" << output.angular_velocity.z << std::endl;
    }
}

void Bridge::fm2chiefcmd(fm::FormationMsg& input, custom_messages::chief_cmd& output) {
    output.name = input.chief_cmd_().name_();
    output.time = input.chief_cmd_().time_();
    output.B_state = input.chief_cmd_().b_formatio_state_();
    output.C_state = input.chief_cmd_().c_formatio_state_();
    output.B_target.pose.pitch = input.chief_cmd_().b_follow_information_().target_information_().pose_().pitch();
    output.B_target.pose.roll = input.chief_cmd_().b_follow_information_().target_information_().pose_().roll();
    output.B_target.pose.yaw = input.chief_cmd_().b_follow_information_().target_information_().pose_().yaw();
    output.B_target.position.x = input.chief_cmd_().b_follow_information_().target_information_().position_().x();
    output.B_target.position.y = input.chief_cmd_().b_follow_information_().target_information_().position_().y();
    output.B_target.position.z = input.chief_cmd_().b_follow_information_().target_information_().position_().z();
    output.B_follow_distance = input.chief_cmd_().b_follow_information_().follow_distance_();
    output.C_target.pose.pitch = input.chief_cmd_().c_follow_information_().target_information_().pose_().pitch();
    output.C_target.pose.roll = input.chief_cmd_().c_follow_information_().target_information_().pose_().roll();
    output.C_target.pose.yaw = input.chief_cmd_().c_follow_information_().target_information_().pose_().yaw();
    output.C_target.position.x = input.chief_cmd_().c_follow_information_().target_information_().position_().x();
    output.C_target.position.y = input.chief_cmd_().c_follow_information_().target_information_().position_().y();
    output.C_target.position.z = input.chief_cmd_().c_follow_information_().target_information_().position_().z();
    output.C_follow_distance = input.chief_cmd_().c_follow_information_().follow_distance_();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "bridge");
    ros::NodeHandle n;

    int set_port;
    bool log_mode;
    n.param<int>("set_port", set_port, 8001);
    n.param<bool>("log_mode", log_mode, false);

    Bridge* myBrige = new Bridge();
    myBrige->log_mode_ = log_mode;

    myBrige->number1_pub = n.advertise<custom_messages::status15>("number1/odom15", 10);
    myBrige->number2_pub = n.advertise<custom_messages::status15>("number2/odom15", 10);
    myBrige->number3_pub = n.advertise<custom_messages::status15>("number3/odom15", 10);
    myBrige->chief_pub = n.advertise<custom_messages::chief_cmd>("chief", 10);

    myBrige->number1_vis_pub = n.advertise<nav_msgs::Odometry>("number1/odometry", 10);
    myBrige->number2_vis_pub = n.advertise<nav_msgs::Odometry>("number2/odometry", 10);
    myBrige->number3_vis_pub = n.advertise<nav_msgs::Odometry>("number3/odometry", 10);

    unsigned short port = set_port;
    // if (argc > 1) {
    //     port = atoi(argv[1]);
    // }
    int sock;
    char buffer[1024] = {0}; 

    myBrige->init(port, sock);
    while(1){
        myBrige->run(sock, buffer);
    }
}