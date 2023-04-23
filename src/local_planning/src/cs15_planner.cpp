#include "cs15.h"

#include <sys/socket.h>
#include <arpa/inet.h>

#include "proto/fm.pb.h"

namespace fm = yw::v2xclient::proto;


// launch info
double cs_15_hz;
double overall_length;
double cspoint_offset = 0.1;
double only_ego_offset;
double avoid_obs_sample_step;
double standard_step;
int avoid_dynamic_mode, avoid_static_mode;
double merge_lateral_threshold, merge_yaw_threshold;

// ros related
ros::Publisher trajectory_pub, trajectory_vis_pub, C_target_pub, number2request_pub;
geometry_msgs::PoseArray trajectory_vis;

// map related
AstarPathFinder* map = new AstarPathFinder();
pcl::PointCloud<pcl::PointXY>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXY>);
pcl::search::KdTree<pcl::PointXY>::Ptr tree (new pcl::search::KdTree<pcl::PointXY>);

inline void coneTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d test)
{
    po[0] = pi[0] * cos(test[2]) - pi[1] * sin(test[2]) + test[0];
    po[1] = pi[0] * sin(test[2]) + pi[1] * cos(test[2]) + test[1];
}

void CScore15::Status15toRosodom(const custom_messages::status15::ConstPtr& input, SplineCarState& output) {
    output.x = input->position.x;
    output.y = input->position.y;
    output.yaw = input->pose.yaw;
    output.speed = input->linear_velocity.x;
    // output.angular_speed = input->angular_velocity.z;
}

SplineCarState & SplineCarState::operator = (const SplineCarState &input) {
    x = input.x;
    y = input.y;
    yaw = input.yaw;
    speed = input.speed;
    angular_speed = input.angular_speed;
}

void CScore15::ReconfigurationCallback(const std_msgs::Int32::ConstPtr& reconfiguration_msg){ // x: 1-add, 2-quit, y: ID
    if (ID == 1) {
        request_numnber2_ = reconfiguration_msg->data;
        ROS_WARN("number2 get reconfiguration_msg:%i", request_numnber2_);
    }
}

void CScore15::ACallback(const custom_messages::status15::ConstPtr& p_msg){
    std::cout << "[cs15]: get A loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    Status15toRosodom(p_msg, fmodom[0]);
    if (ID == 0) {
        // self_initial = true;
        ROS_ERROR("emlike module`s ID cant be 0");
    } else if (target_ID == 0) {
        front_initial = true;
    }
}
void CScore15::BCallback(const custom_messages::status15::ConstPtr& p_msg){
    std::cout << "[cs15]: get B loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    Status15toRosodom(p_msg, fmodom[1]);
    if (ID == 1) {
        self_initial = true;
    } else if (target_ID == 1) {
        front_initial = true;
    }
    if (p_msg->request == 2) {
        if (ID == 2) {
            add_reconfiguration_flag = true;
        }
        reconfiguration_flag = 1;
    } else if (p_msg->request == 1) {
        if (ID == 2) {
            target_ID = 0;
        }
        reconfiguration_flag = 2;
    }
}
void CScore15::CCallback(const custom_messages::status15::ConstPtr& p_msg){
    std::cout << "[cs15]: get C loc" << std::endl;
    // printf("idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    Status15toRosodom(p_msg, fmodom[2]);
    if (ID == 2) {
        self_initial = true;
    } else if (target_ID == 2) {
        // front_initial = true;
        ROS_ERROR("emlike module`s targetID cant be 2");
    }
}



// bool myCS15::GetVehicleInfo(int mode, int index, geometry_msgs::PoseStamped& out) {
//     if (index < 0) {
//         return false;
//     }
//     if (pList[index].first == 0) {
//         // printf("[cs_planner]: get %d info failed\n", index);
//         return false;
//     }
//     nav_msgs::Odometry tmp(pList[index].second);
//     out.pose.position.x = tmp.pose.pose.position.x;
//     out.pose.position.y = tmp.pose.pose.position.y;
//     out.pose.orientation.w = tmp.pose.pose.orientation.w;
//     out.pose.orientation.x = tmp.pose.pose.orientation.x;
//     out.pose.orientation.y = tmp.pose.pose.orientation.y;
//     out.pose.orientation.z = tmp.pose.pose.orientation.z;
//     tf::Quaternion RQ2;
//     double roll,pitch,yaw;
//     tf::quaternionMsgToTF(tmp.pose.pose.orientation,RQ2);  
//     tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
//     if (mode == 0) {
//         front_yaw = yaw;
//         front_initial = true;
//     } else if (mode == 1) {
//         leader_yaw = yaw;  
//         leader_initial = true;
//     }
//     return true;
// }
// bool myCS15::GetVehiclemyInfo(int index, SplineCarState& out) {
//     if (pList[index].first == 0) {
//         // printf("[cs_planner]: get self info failed\n");
//         return false;
//     }
//     nav_msgs::Odometry tmp(pList[index].second);
//     out.x = tmp.pose.pose.position.x;
//     out.y = tmp.pose.pose.position.y;
//     tf::Quaternion RQ2;
//     double roll,pitch,yaw;
//     tf::quaternionMsgToTF(tmp.pose.pose.orientation,RQ2);  
//     tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  
//     out.yaw = yaw;
//     self_initial = true;
//     return true;
// }

bool CScore15::add_check() {
    if (add_yaw_check() && add_lateral_check()) {
        return true;
    } else {
        return false;
    }
}

bool CScore15::add_lateral_check() {
    if (abs(fmodom[0].yaw - M_PI / 2) < 1e-6 || abs(fmodom[0].yaw + M_PI / 2) < 1e-6) {
        double lateral_error = abs(fmodom[0].x - fmodom[1].x);
        ROS_WARN("[cs15]: add_recfg lateral_error = %f(%f)", lateral_error, merge_lateral_threshold);
        if (lateral_error > merge_lateral_threshold) {
            return false;
        }
    } else {
        double k = tan(fmodom[0].yaw);
        double A = k;
        double B = -1.0;
        double C = fmodom[0].y - k * fmodom[0].x;
        double x = fmodom[1].x;
        double y = fmodom[1].y;
        double lateral_d = sqrt(pow((A * x + B * y + C), 2)) / sqrt(pow(A, 2) + pow(B, 2));
        ROS_WARN("[cs15]: add_recfg lateral_error = %f(%f)", lateral_d, merge_lateral_threshold);
        if (lateral_d > merge_lateral_threshold) {
            return false;
        }
    }
    return true;
}

bool CScore15::add_yaw_check() {
    double yaw_error = abs(fmodom[0].yaw - fmodom[1].yaw);
    ROS_WARN("[cs15]: add_recfg yaw_error = %f(%f)", yaw_error, merge_yaw_threshold);
    if (yaw_error > merge_yaw_threshold) {
        return false;
    } else {
        return true;
    }
}

void CScore15::ObsCallback(const custom_messages::mapmodel::ConstPtr& obs_msg){
    if(!front_initial || obs_msg->mapinfo.empty()){
        return;
    }
    obstacle.mapinfo.clear();
    for(size_t i = 0; i < obs_msg->mapinfo.size(); ++i){
        double FLtarget2obs = sqrt(pow(obs_msg->mapinfo[i].x - front.x, 2) + pow(obs_msg->mapinfo[i].y - front.y, 2));
        if(FLtarget2obs > obs_msg->mapinfo[i].z + overall_length/2){
            obstacle.mapinfo.push_back(obs_msg->mapinfo[i]);
        }
    }
}

void AstarPathFinder::SlamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& slam_map_msgs)
{
    if(!_slam_map.data.empty() || slam_map_msgs == nullptr || slam_map_msgs->data.empty()) return;

    _slam_map = *slam_map_msgs;

    GLX_SIZE = _slam_map.info.width;
    GLY_SIZE = _slam_map.info.height;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE; 
    resolution = _slam_map.info.resolution;
    inv_resolution = 1 / resolution;
    gl_xl = _slam_map.info.origin.position.x;
    gl_yl = _slam_map.info.origin.position.y;
    gl_xu = gl_xl + GLX_SIZE * resolution;
    gl_yu = gl_yl + GLY_SIZE * resolution;

	// ofstream out("/home/csxr/out1.txt", ios::app);
    // for (int j = 0; j <_slam_map.data.size(); ++j) {
    //     out<<(int)_slam_map.data[j]<<" ";
    // }
    map->inflation();
	// ofstream out2("/home/csxr/out2.txt", ios::app);
    // for (int j = 0; j <_slam_map.data.size(); ++j) {
    //     out2<<(int)_slam_map.data[j]<<" ";
    // }
    initGridMap();

    for(int i = 0; i < GLX_SIZE; ++i)
    {
        for(int j = 0; j < GLY_SIZE; ++j)
        {
            if(isOccupied(i,j)){
                pcl::PointXY p;
                Eigen::Vector2i tmpi;
                tmpi << i, j;
                Eigen::Vector2d tmpd = gridIndex2coord(tmpi);
                p.x = tmpd(0);
                p.y = tmpd(1);
                mapCloud->push_back(p);
            }
        }
    }
    tree->setInputCloud(mapCloud);
}

bool CScore15::CollisionCheck(const CSRefPath &r_path, const SplineCarState &carstate, const AstarPathFinder* tmpmap, int dynamic_button, int static_button)
{
    double distance;
    if(dynamic_button){
        for (int i = 0; i < obstacle.mapinfo.size(); i ++) {
            for (int j = 4; j < r_path.r_s.size(); j ++) { // j=4,because its near is already safe
                distance = sqrt(pow(obstacle.mapinfo[i].x - r_path.r_x[j], 2) + pow(obstacle.mapinfo[i].y - r_path.r_y[j], 2));
                if(distance <= obstacle.mapinfo[i].z + overall_length){
                    std::cout << "[cs15] : obs collision" << std::endl;
                    return true;
                }
            }
        }
    }

    if (static_button) {
        double window = 3.0 + carstate.speed;
        std::cout << "map size: " << mapCloud->size() << std::endl;
        if (mapCloud->size() < 1) {
            return false;
        }
        // cout << mapCloud->points[0].x << mapCloud->points[0].y << endl;
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        pcl::PointXY pCar;
        pCar.x = carstate.x;
        pCar.y = carstate.y;
        // std::cout << "pCar:" << pCar.x << "," << pCar.y << std::endl;
        int num = tree->radiusSearch(pCar, window, k_indices, k_sqr_distances);
        // cout << "num: " << num << endl;
        if (num) {
            for (size_t i (0); i < (k_indices).size(); i++) {
                pcl::PointXY p = mapCloud->points[(k_indices)[i]];
                // cout << "k_dis: " << k_sqr_distances[i] << endl;
                for (int k = 0; k < r_path.r_s.size(); k++) {
                    distance = sqrt(pow(p.x - r_path.r_x[k], 2) + pow(p.y - r_path.r_y[k], 2));
                    if(distance <= 0.1){
                        std::cout << "[cs15] : map collision" << std::endl;
                        return true;
                    }
                }     
            }
        }
        return false;
    } else {
        return false;
    }

}

void CScore15::planning(){
    // wait for initial
    if (!(self_initial && front_initial)) {
        if (!self_initial) {
            ROS_WARN("[cs15]: loc for ego NOT init");
        }
        if (!front_initial) {
            ROS_WARN("[cs15]: loc for front NOT init");
        }

        custom_messages::planning_info empty2control;
        trajectory_pub.publish(empty2control);
        keeper.clear();
        return;
    }

    // add check
    if (add_reconfiguration_flag) {
        if (!add_check()) {
            ROS_WARN("follow old number1");
            if (ID == 2) {
                target_ID = 0;
            }
        } else {
            ROS_WARN("follow new number2");
            add_reconfiguration_flag = false;
            if (ID == 2) {
                target_ID = 1;
            }
        }
    }

    front = fmodom[target_ID];
    SplineCarState target = front;
    mystate = fmodom[ID];

    CSRefPath newCSRefPath;
    if (!keeper.empty()) {
        std::cout << "[cs15]: trajectory jointing" << std::endl;
        Eigen::Vector2d last_trajectory_front(keeper[0].x, keeper[0].y);
        Eigen::Vector2d last_trajectory_front_offset(keeper[0].x + only_ego_offset * cos(keeper[0].yaw), keeper[0].y + only_ego_offset * sin(keeper[0].yaw));
        Eigen::Vector2d last_trajectory_back(keeper[1].x, keeper[1].y);
        Eigen::Vector2d last_trajectory_back_offset4last(keeper[1].x - cspoint_offset * cos(keeper[1].yaw), keeper[1].y - cspoint_offset * sin(keeper[1].yaw));
        Eigen::Vector2d last_trajectory_back_offset4now(keeper[1].x + cspoint_offset * cos(keeper[1].yaw), keeper[1].y + cspoint_offset * sin(keeper[1].yaw));
        Eigen::Vector2d last_trajectory_mid((keeper[0].x + keeper[1].x) * 0.5, (keeper[0].y + keeper[1].y) * 0.5);
        Eigen::Vector2d target_offset(target.x - cspoint_offset * cos(target.yaw), target.y - cspoint_offset * sin(target.yaw));

        Eigen::Vector2d front_move_flag(target_offset[0] - last_trajectory_back_offset4now[0], target_offset[1] - last_trajectory_back_offset4now[1]);
        double move_angle = atan2(front_move_flag[1], front_move_flag[0]);
        std::cout << "moveflag:x=" << front_move_flag[0] << ",y=" << front_move_flag[1] << std::endl;
        std::cout << "moveyaw=" << move_angle << ",front_yaw=" << front.yaw << std::endl;
        bool is_backward = (abs(move_angle - front.yaw) > M_PI / 2) ? true : false;
        if (!is_backward) {
            std::cout << "[cs15]: target is moving" << std::endl;
            std::vector<double> newsegmentx({last_trajectory_front[0], last_trajectory_front_offset[0], last_trajectory_mid[0], last_trajectory_back_offset4last[0], last_trajectory_back[0], last_trajectory_back_offset4now[0], target_offset[0], target.x});
            std::vector<double> newsegmenty({last_trajectory_front[1], last_trajectory_front_offset[1], last_trajectory_mid[1], last_trajectory_back_offset4last[1], last_trajectory_back[1], last_trajectory_back_offset4now[1], target_offset[1], target.y});
            Spline2D newspline(newsegmentx, newsegmenty);
            newCSRefPath.load(newspline, standard_step);
        } else {
            std::cout << "[cs15]: target is NOT moving!!!" << std::endl;
            std::vector<double> newsegmentx({last_trajectory_front[0], last_trajectory_front_offset[0], last_trajectory_mid[0], target_offset[0], target.x});
            std::vector<double> newsegmenty({last_trajectory_front[1], last_trajectory_front_offset[1], last_trajectory_mid[1], target_offset[1], target.y});
            Spline2D newspline(newsegmentx, newsegmenty);
            newCSRefPath.load(newspline, standard_step);
        }

        if (!CollisionCheck(newCSRefPath, mystate, map, avoid_dynamic_mode, avoid_static_mode)) {
            std::cout << "[cs15]: joint result publish" << std::endl;
            trajectory_vis.poses.clear();
            trajectory_vis.header.frame_id = "map";
            trajectory_vis.header.stamp = ros::Time::now();
            geometry_msgs::Pose tmppose;

            custom_messages::planning_info trajectory2control;
            custom_messages::vehicle_status plan_pose0;
            int num0 = newCSRefPath.r_s.size();
            for(int i = 0; i < num0; ++i) {
                plan_pose0.xPos = newCSRefPath.r_x[i];
                plan_pose0.yPos = newCSRefPath.r_y[i];
                plan_pose0.yaw = newCSRefPath.r_yaw[i];
                plan_pose0.speed = newCSRefPath.r_s[i];
                plan_pose0.angular_speed = newCSRefPath.r_curvature[i];
                trajectory2control.states.emplace_back(plan_pose0);
                std::cout << "[cs_planner]: pub_tmp_x = " << newCSRefPath.r_x[i] << ",tmp_y = " << newCSRefPath.r_y[i] << std::endl;
                std::cout << "[cs_planner]: pub_tmp_s = " << newCSRefPath.r_s[i] << "[" << i << "]" << std::endl;

                tmppose.position.x = newCSRefPath.r_x[i];
                tmppose.position.y = newCSRefPath.r_y[i];
                double tmpyaw = newCSRefPath.r_yaw[i];
                //输入欧拉角，转化成四元数在终端输出
                geometry_msgs::Quaternion q;
                q=tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpyaw);
                tmppose.orientation.w = q.w;
                tmppose.orientation.x = q.x;
                tmppose.orientation.y = q.y;
                tmppose.orientation.z = q.z;
                trajectory_vis.poses.emplace_back(tmppose);
            }
            trajectory_pub.publish(trajectory2control);
            trajectory_vis_pub.publish(trajectory_vis);
            UpdateKeeper();
            return;
        }
        std::cout << "[cs15]: joint failed, generate new one" << std::endl; 
    }

    // construct CSkeypoint
    // double distance = sqrt( pow((target.x - mystate.x), 2) + pow((target.y - mystate.y), 2));
    double slope = (target.y - mystate.y) / (target.x - mystate.x);
    // use perpendicular_k may cause it equal to nan, so it need method isnan to make sure its safe
    // double perpendicular_k = - 1 / slope; 

    double perpendicular_angle = atan2(-(target.x - mystate.x), (target.y - mystate.y));
    double selfnext_x = mystate.x + cspoint_offset * cos(mystate.yaw);
    double selfnext_y = mystate.y + cspoint_offset * sin(mystate.yaw);
    double frontpre_x = target.x - cspoint_offset * cos(target.yaw);
    double frontpre_y = target.y - cspoint_offset * sin(target.yaw);
    Eigen::Vector2d midpoint;
    midpoint << (selfnext_x + frontpre_x) / 2, (selfnext_y + frontpre_y) / 2;
    std::vector<double> wx({mystate.x, selfnext_x, midpoint(0), frontpre_x, target.x});
    std::vector<double> wy({mystate.y, selfnext_y, midpoint(1), frontpre_y, target.y});

    // loop to generate safe trajectory
    int count = 0;
    CSRefPath r_path1;
    CSRefPath r_path2;
    std::vector<double> wx1(wx);
    std::vector<double> wy1(wy);
    std::vector<double> wx2(wx);
    std::vector<double> wy2(wy);
    while(count <= 50){
        std::cout << "[cs15] : spline midpoint sample count=" << count << std::endl;
        r_path1.CSclear();
        r_path2.CSclear();
        Spline2D path1(wx1,wy1);
        Spline2D path2(wx2,wy2);
        r_path1.load(path1, standard_step);
        r_path2.load(path2, standard_step);

        if(CollisionCheck(r_path1, mystate, map, avoid_dynamic_mode, avoid_static_mode)){
            double tmpx1, tmpy1;
            //  midperpendicular function : y = - x / k + (x1 + x2) / (2*k) + (y1 + y2) / 2

            tmpx1 = midpoint(0) + count * avoid_obs_sample_step * abs(cos(perpendicular_angle));
            tmpy1 = - tmpx1 / slope + midpoint(0) / slope + midpoint(1); 

            wx1[2] = tmpx1;
            wy1[2] = tmpy1;
            // std::cout << "tmp1:x=" << tmpx1 << ", y=" << tmpy1 << std::endl;
        }
        else{
            trajectory_vis.poses.clear();
            trajectory_vis.header.frame_id = "map";
            trajectory_vis.header.stamp = ros::Time::now();
            geometry_msgs::Pose tmppose;

            custom_messages::planning_info trajectory2control;
            custom_messages::vehicle_status plan_pose0;
            int num0 = r_path1.r_s.size();
            for(int i = 0; i < num0; i++)
            {
                plan_pose0.xPos = r_path1.r_x[i];
                plan_pose0.yPos = r_path1.r_y[i];
                plan_pose0.yaw = r_path1.r_yaw[i];
                plan_pose0.speed = r_path1.r_s[i];
                plan_pose0.angular_speed = r_path1.r_curvature[i];
                trajectory2control.states.push_back(plan_pose0);

                tmppose.position.x = r_path1.r_x[i];
                tmppose.position.y = r_path1.r_y[i];
                double tmpyaw = r_path1.r_yaw[i];
                //输入欧拉角，转化成四元数在终端输出
                geometry_msgs::Quaternion q;
                q=tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpyaw);
                tmppose.orientation.w = q.w;
                tmppose.orientation.x = q.x;
                tmppose.orientation.y = q.y;
                tmppose.orientation.z = q.z;
                trajectory_vis.poses.emplace_back(tmppose);
            }
            trajectory_pub.publish(trajectory2control);
            trajectory_vis_pub.publish(trajectory_vis);
            UpdateKeeper();
            return;
        }
        if(CollisionCheck(r_path2, mystate, map, avoid_dynamic_mode, avoid_static_mode)){
            double tmpx2, tmpy2;

            tmpx2 = midpoint(0) - count * avoid_obs_sample_step * abs(cos(perpendicular_angle));
            tmpy2 = - tmpx2 / slope + midpoint(0) / slope + midpoint(1);

            wx2[2] = tmpx2;
            wy2[2] = tmpy2;
            // std::cout << "tmp2:x=" << tmpx2 << ", y=" << tmpy2 << std::endl;
        }
        else{
            trajectory_vis.poses.clear();
            trajectory_vis.header.frame_id = "map";
            trajectory_vis.header.stamp = ros::Time::now();
            geometry_msgs::Pose tmppose;

            custom_messages::planning_info trajectory2control;
            custom_messages::vehicle_status plan_pose0;
            int num0 = r_path2.r_s.size();
            for(int i = 0; i < num0; i++)
            {
                plan_pose0.xPos = r_path2.r_x[i];
                plan_pose0.yPos = r_path2.r_y[i];
                plan_pose0.yaw = r_path2.r_yaw[i];
                plan_pose0.speed = r_path2.r_s[i];
                plan_pose0.angular_speed = r_path2.r_curvature[i];
                trajectory2control.states.push_back(plan_pose0);

                tmppose.position.x = r_path2.r_x[i];
                tmppose.position.y = r_path2.r_y[i];
                double tmpyaw = r_path2.r_yaw[i];
                //输入欧拉角，转化成四元数在终端输出
                geometry_msgs::Quaternion q;
                q=tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpyaw);
                tmppose.orientation.w = q.w;
                tmppose.orientation.x = q.x;
                tmppose.orientation.y = q.y;
                tmppose.orientation.z = q.z;
                trajectory_vis.poses.emplace_back(tmppose);
            }
            trajectory_pub.publish(trajectory2control);
            trajectory_vis_pub.publish(trajectory_vis);
            UpdateKeeper();
            return;
        }
        ++count;
    }
    std::cout << "[cs15] : spline_planner failed" << std::endl;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cs_15_planner");
    ros::NodeHandle nh;

    int recv_ID, recv_target_ID, request_numnber2;
    nh.param<int>("recv_ID", recv_ID, 1);
    nh.param<int>("recv_target_ID", recv_target_ID, 0);
    nh.param<double>("cs_15_planner/cs15_hz", cs_15_hz, 10.0);
    nh.param<double>("overall_length", overall_length, 0.7);
    nh.param<double>("cs_15_planner/avoid_obs_sample_step", avoid_obs_sample_step, 0.1);
    nh.param<int>("cs_15_planner/avoid_dynamic_mode", avoid_dynamic_mode, 0);
    nh.param<int>("cs_15_planner/avoid_static_mode", avoid_static_mode, 0);
    nh.param<double>("merge_lateral_threshold", merge_lateral_threshold, 1.5);
    nh.param<double>("merge_yaw_threshold", merge_yaw_threshold, M_PI / 12);
    nh.param<int>("request_numnber2", request_numnber2, 0);
    nh.param<double>("cs_15_planner/standard_step", standard_step, 0.5);
    nh.param<double>("cs_15_planner/only_ego_offset", only_ego_offset, 2.0);

    trajectory_pub = nh.advertise<custom_messages::planning_info>("trajectory_cs15", 1, true);
    trajectory_vis_pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_cs15_vis", 1, true);
    C_target_pub = nh.advertise<std_msgs::Int32>("/C_target", 1, true);
    number2request_pub = nh.advertise<std_msgs::Int32>("/fixpb4number2request", 1, true);

    CScore15* myCS15 = new CScore15(recv_ID, recv_target_ID);
    myCS15->request_numnber2_ = request_numnber2;

    ros::Subscriber reconfiguration_sub = nh.subscribe("/reID", 1, &CScore15::ReconfigurationCallback, myCS15);

    ros::Subscriber A_sub = nh.subscribe("/number1/odom15", 10, &CScore15::ACallback, myCS15);
    ros::Subscriber B_sub = nh.subscribe("/number2/odom15", 10, &CScore15::BCallback, myCS15);
    ros::Subscriber C_sub = nh.subscribe("/number3/odom15", 10, &CScore15::CCallback, myCS15);
    // ros::Subscriber chief_sub = nh.subscribe("chief", 10, &CScore15::ReconfigurationCallback, myCS15)

    ros::Subscriber map_sub = nh.subscribe("slam_ret", 1,  &AstarPathFinder::SlamMapCallback, map);
    ros::Subscriber obs_sub = nh.subscribe("obstacle_info", 1, &CScore15::ObsCallback, myCS15);

    ros::Rate rate(cs_15_hz);
    while(ros::ok()){
        myCS15->reconfiguration_flag = 0;

        ros::spinOnce();

        std::cout << "[cs15]: EGO NAME: " << myCS15->ID << std::endl;
        std::cout << "[cs15]: TARGET NAME: " << myCS15->target_ID << std::endl;

        // ros::Time time1 = ros::Time::now();
        myCS15->planning();
        // ros::Time time2 = ros::Time::now();
        // std::printf("[cs planner]: Time consume is : %f\n",(time2 - time1).toSec() );

        // pub for monitor
        std_msgs::Int32 id_flag;
        id_flag.data = myCS15->target_ID;
        // ROS_WARN("check if i am number2 publish C_target");
        if (myCS15->ID == 2) {
            // ROS_WARN("publish C_target");
            C_target_pub.publish(id_flag);
        }

        rate.sleep();
    }
}