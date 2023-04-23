#include "cubic_spline.h"

// launch info
int ID;
double cubic_spline_hz;
double overall_length;
double cspoint_offset = 0.1;
double standard_step;
int avoid_mode;
int formation_type;
bool formation_change = false;

// reconfiguration function
std::vector<std::pair<int, nav_msgs::Odometry>> pList(10);
int reconfiguration_flag; // 0-false, 1-add(particular unit), 2-quit
bool add_reconfiguration_flag = false;
double add_lateral_threshold, add_yaw_threshold;
// formation change
double formation_length_unit;

// ros related
ros::Publisher trajectory_vis_pub;
geometry_msgs::PoseArray trajectory_vis;

// map related
AstarPathFinder* map = new AstarPathFinder();
pcl::PointCloud<pcl::PointXY>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXY>);
pcl::search::KdTree<pcl::PointXY>::Ptr tree (new pcl::search::KdTree<pcl::PointXY>);

void ClearpList() {
    for (auto & x : pList) {
        x.first = 0;
    }
}

inline void coneTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d test)
{
    po[0] = pi[0] * cos(test[2]) - pi[1] * sin(test[2]) + test[0];
    po[1] = pi[0] * sin(test[2]) + pi[1] * cos(test[2]) + test[1];
}

void CubicSplineClass::PlatoonCallback(const nav_msgs::Odometry::ConstPtr& p_msg){
    int idx = p_msg->pose.pose.position.z;
    printf("[cs_planner]: locCallback idx=%i, x=%f, y=%f\n", idx, p_msg->pose.pose.position.x, p_msg->pose.pose.position.y);
    pList[idx].first = 1;
    pList[idx].second = *p_msg;
}

bool CubicSplineClass::GetVehicleInfo(int mode, int index, geometry_msgs::PoseStamped& out) {
    if (index < 0) {
        return false;
    }
    if (pList[index].first == 0) {
        // printf("[cs_planner]: get %d info failed\n", index);
        return false;
    }
    nav_msgs::Odometry tmp(pList[index].second);
    out.pose.position.x = tmp.pose.pose.position.x;
    if (isnan(tmp.pose.pose.position.x)) {
        ROS_WARN("No_%i loc x exception", index);
        return false;
    }

    out.pose.position.y = tmp.pose.pose.position.y;
    if (isnan(tmp.pose.pose.position.y)) {
        ROS_WARN("No_%i loc y exception", index);
        return false;
    }
    out.pose.orientation.w = tmp.pose.pose.orientation.w;
    out.pose.orientation.x = tmp.pose.pose.orientation.x;
    out.pose.orientation.y = tmp.pose.pose.orientation.y;
    out.pose.orientation.z = tmp.pose.pose.orientation.z;
    tf::Quaternion RQ2;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(tmp.pose.pose.orientation,RQ2);  
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
    if (isnan(yaw)) {
        ROS_WARN("No_%i loc yaw exception", index);
        return false;
    }
    if (mode == 0) {
        front_yaw = yaw;
        front_initial = true;
    } else if (mode == 1) {
        leader_yaw = yaw;  
        leader_initial = true;
    }
    return true;
}
bool CubicSplineClass::GetVehiclemyInfo(int index, SplineCarState& out) {
    if (pList[index].first == 0) {
        // printf("[cs_planner]: get self info failed\n");
        return false;
    }
    nav_msgs::Odometry tmp(pList[index].second);
    out.x = tmp.pose.pose.position.x;
    if (isnan(tmp.pose.pose.position.x)) {
        ROS_WARN("No_%i loc x exception", index);
        return false;
    }
    out.y = tmp.pose.pose.position.y;
    if (isnan(tmp.pose.pose.position.y)) {
        ROS_WARN("No_%i loc y exception", index);
        return false;
    }
    tf::Quaternion RQ2;
    double roll,pitch,yaw;
    tf::quaternionMsgToTF(tmp.pose.pose.orientation,RQ2);  
    tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
    if (isnan(yaw)) {
        ROS_WARN("No_%i loc yaw exception", index);
        return false;
    }
    out.yaw = yaw;
    self_initial = true;
    return true;
}

bool CubicSplineClass::add_check(SplineCarState &last_front) {
    if (add_yaw_check(last_front) && add_lateral_check(last_front)) {
        return true;
    } else {
        return false;
    }
}

bool CubicSplineClass::add_lateral_check(SplineCarState &last_front) {
    if (abs(last_front.yaw - M_PI / 2) < 1e-6 || abs(last_front.yaw + M_PI / 2) < 1e-6) {
        double lateral_error = abs(last_front.x - front.pose.position.x);
        ROS_WARN("[cs_planner]: add_recfg lateral_error = %f(%f)", lateral_error, add_lateral_threshold);
        if (lateral_error > add_lateral_threshold) {
            return false;
        }
    } else {
        double k = tan(last_front.yaw);
        double A = k;
        double B = -1.0;
        double C = last_front.y - k * last_front.x;
        double x = front.pose.position.x;
        double y = front.pose.position.y;
        double lateral_d = sqrt(pow((A * x + B * y + C), 2)) / sqrt(pow(A, 2) + pow(B, 2));
        ROS_WARN("[cs_planner]: add_recfg lateral_error = %f(%f)", lateral_d, add_lateral_threshold);
        if (lateral_d > add_lateral_threshold) {
            return false;
        }
    }
    return true;
}

bool CubicSplineClass::add_yaw_check(SplineCarState &last_front) {
    double yaw_error = abs(last_front.yaw - front_yaw);
    ROS_WARN("[cs_planner]: add_recfg yaw_error = %f(%f)", yaw_error, add_yaw_threshold);
    if (yaw_error > add_yaw_threshold) {
        return false;
    } else {
        return true;
    }
}

void CubicSplineClass::ReconfigurationCallback(const geometry_msgs::Point::ConstPtr& reconstruction_msg){ // x: 1-add, 2-quit, y: ID
    int mode = reconstruction_msg->x;
    int reID = reconstruction_msg->y;
    if (mode == 1) {
        if (ID == 4) {
            std::cout << "[cs_planner]: I am adding" << std::endl;
            ID = reID;
        } else if (ID == reID) {
            std::cout << "[cs_planner] add mode" << std::endl;
            ++ID;
            add_reconfiguration_flag = true;
        } else if(ID > reID) {
            std::cout << "[cs_planner] add mode" << std::endl;
            ++ID;
        }
        reconfiguration_flag = 1;
    } else if (mode == 2) {
        if (ID == reID) {
            ROS_FATAL("I'm %i , I am quitting", ID);
            ros::shutdown();
        } else if (ID > reID) {
            std::cout << "[cs_planner] quit mode" << std::endl;
            --ID;
        }
        reconfiguration_flag = 2;
    } else {
        ROS_WARN("wrong reconfiguration mode");
    }
}

void CubicSplineClass::ObsCallback(const custom_messages::mapmodel::ConstPtr& obs_msg){
    if(!front_initial || obs_msg->mapinfo.empty()){
        return;
    }
    obstacle.mapinfo.clear();
    for(size_t i = 0; i < obs_msg->mapinfo.size(); ++i){
        double FLtarget2obs = sqrt(pow(obs_msg->mapinfo[i].x - front.pose.position.x, 2) + pow(obs_msg->mapinfo[i].y - front.pose.position.y, 2));
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

bool CubicSplineClass::CollisionCheck(const CSRefPath &r_path, const SplineCarState &carstate, const AstarPathFinder* tmpmap, int mode)
{
    double distance;
    if(mode){
        for (int i = 0; i < obstacle.mapinfo.size(); i ++) {
            for (int j = 4; j < r_path.r_s.size(); j ++) { // j=4,because its near is already safe
                distance = sqrt(pow(obstacle.mapinfo[i].x - r_path.r_x[j], 2) + pow(obstacle.mapinfo[i].y - r_path.r_y[j], 2));
                if(distance <= obstacle.mapinfo[i].z + overall_length){
                    std::cout << "[local_planning] : obs collision" << std::endl;
                    return true;
                }
            }
        }
    }

    double window = 3.0 + carstate.speed;
    // cout << "map size: " << mapCloud->size() << endl;
    // cout << mapCloud->points[0].x << mapCloud->points[0].y << endl;
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    pcl::PointXY pCar;
    pCar.x = carstate.x;
    pCar.y = carstate.y;
    int num = tree->radiusSearch(pCar, window, k_indices, k_sqr_distances);
    // cout << "num: " << num << endl;
    if (num) {
        for (size_t i (0); i < (k_indices).size(); i++) {
            pcl::PointXY p = mapCloud->points[(k_indices)[i]];
            // cout << "k_dis: " << k_sqr_distances[i] << endl;
            for (int k = 0; k < r_path.r_s.size(); k++) {
                distance = sqrt(pow(p.x - r_path.r_x[k], 2) + pow(p.y - r_path.r_y[k], 2));
                if(distance <= 0.1){
                    std::cout << "[local_planning] : map collision" << std::endl;
                    return true;
                }
            }     
        }
    }
    return false;
}

void CubicSplineClass::update_formation(double formation_length_unit) {
    /*             0
            left       right
                  tail 
    */
    formation_diamond[0] << 0, 0;
    Eigen::Vector3d move;
    move << leader.pose.position.x, leader.pose.position.y, leader_yaw; 
    Eigen::Vector2d point_in_leader;

    point_in_leader << -1.0 * formation_length_unit + 0.2, 1.0 * formation_length_unit;
    coneTrans(point_in_leader, formation_diamond[1], move);

    point_in_leader(0) = -1.0 * formation_length_unit + 0.2;
    point_in_leader(1) = -1.0 * formation_length_unit;
    coneTrans(point_in_leader, formation_diamond[2], move);

    point_in_leader(0) = -2.0 * formation_length_unit + 0.2;
    point_in_leader(1) = 0.0;
    coneTrans(point_in_leader, formation_diamond[3], move);

    /*        0
        1     3     2
    */
    formation_triangle[0] << 0, 0;

    point_in_leader << -1.0 * formation_length_unit + 0.2, 0.5 * formation_length_unit;
    coneTrans(point_in_leader, formation_triangle[1], move);

    point_in_leader(0) = -1.0 * formation_length_unit + 0.2;
    point_in_leader(1) = -0.5 * formation_length_unit;
    coneTrans(point_in_leader, formation_triangle[2], move);

    point_in_leader(0) = -1.0 * formation_length_unit + 0.2;
    point_in_leader(1) = 0.0;
    coneTrans(point_in_leader, formation_triangle[3], move);
}

void CubicSplineClass::planning(){
    // configuration timing to skip this loop
    if (reconfiguration_flag != 0) {
        ROS_WARN("[cs_planner]: reconfiguration trigger");
        return;
    }

    if(GetVehicleInfo(0, ID - 1, front)) {
        std::cout << "[cs_planner]: FRONT X: " << front.pose.position.x << ", Y: " << front.pose.position.y << std::endl;
    } else {
        return;
    }
    if(GetVehicleInfo(1, 0, leader)) {
        std::cout << "[cs_planner]: LEADER X: " << leader.pose.position.x << ", Y: " << leader.pose.position.y << std::endl;
    }
    if (GetVehiclemyInfo(ID, mystate)) {
        std::cout << "[cs_planner]: myX: " << mystate.x << ", myY: " << mystate.y << std::endl;
    } else {
        return;
    }

    // wait for initial
    if (!(self_initial && front_initial)) {
        if (!self_initial) {
            ROS_WARN("[cs_planner]: loc for ego NOT init");
        }
        if (!front_initial) {
            ROS_WARN("[cs_planner]: loc for front NOT init");
        }

        custom_messages::planning_info empty2control;
        _trajectory_pub.publish(empty2control);
        keeper.clear();
        return;
    }
    if (formation_type && !leader_initial) {
        custom_messages::planning_info empty2control;
        _trajectory_pub.publish(empty2control);
        keeper.clear();
        return;
    }

    if (formation_change) {
        keeper.clear();
        formation_change = false;
        return;
    }

    update_formation(formation_length_unit);

    // formation type change and add_reconfiguration
    SplineCarState target;
    switch (formation_type)
    {
    case 0: // regular column
        {
            // add check
            if (add_reconfiguration_flag) {
                SplineCarState last_front;
                GetVehiclemyInfo(ID - 2, last_front);
                if (!add_check(last_front)) {
                    ROS_WARN("[cs_planner]: follow last front");
                    target.x = last_front.x;
                    target.y = last_front.y;
                    target.yaw = last_front.yaw;
                } else {
                    ROS_WARN("[cs_planner]: back to current front");
                    target.x = front.pose.position.x;
                    target.y = front.pose.position.y;
                    target.yaw = front_yaw;
                    add_reconfiguration_flag = false;
                }
            } else {
                std::cout << "[cs_planner]: formation : regular" << std::endl;
                target.x = front.pose.position.x;
                target.y = front.pose.position.y;
                target.yaw = front_yaw;
            }
            break;
        }
    case 1: // diamond
        {
            std::cout << "[cs_planner]: formation: diamond(" << formation_diamond[ID][0] << "," << formation_diamond[ID][1] << ")" << std::endl;
            target.x = formation_diamond[ID][0];
            target.y = formation_diamond[ID][1];
            target.yaw = leader_yaw;
            break;
        }
    case 2: //triangle
        {
            std::cout << "[cs_planner]: formation: triangle(" << formation_triangle[ID][0] << "," << formation_triangle[ID][1] << ")" << std::endl;
            target.x = formation_triangle[ID][0];
            target.y = formation_triangle[ID][1];
            target.yaw = leader_yaw;
            break;
        }
    }

    // version2: keep last loc to joint
    CSRefPath newCSRefPath;
    if (!keeper.empty()) {
        std::cout << "[cs_planner]: trajectory jointing" << std::endl;
        Eigen::Vector2d last_trajectory_front(keeper[0].x, keeper[0].y);
        Eigen::Vector2d last_trajectory_front_offset(keeper[0].x + cspoint_offset * cos(keeper[0].yaw), keeper[0].y + cspoint_offset * sin(keeper[0].yaw));
        Eigen::Vector2d last_trajectory_back(keeper[1].x, keeper[1].y);
        Eigen::Vector2d last_trajectory_back_offset4last(keeper[1].x - cspoint_offset * cos(keeper[1].yaw), keeper[1].y - cspoint_offset * sin(keeper[1].yaw));
        Eigen::Vector2d last_trajectory_back_offset4now(keeper[1].x + cspoint_offset * cos(keeper[1].yaw), keeper[1].y + cspoint_offset * sin(keeper[1].yaw));
        Eigen::Vector2d target_offset(target.x - cspoint_offset * cos(target.yaw), target.y - cspoint_offset * sin(target.yaw));
        Eigen::Vector2d last_trajectory_mid((keeper[0].x + keeper[1].x) * 0.5, (keeper[0].y + keeper[1].y) * 0.5);

        Eigen::Vector2d front_move_flag(target_offset[0] - last_trajectory_back_offset4now[0], target_offset[1] - last_trajectory_back_offset4now[1]);
        double move_angle = atan2(front_move_flag[1], front_move_flag[0]);
        std::cout << "[cs_planner]: moveflag:x=" << front_move_flag[0] << ",y=" << front_move_flag[1] << std::endl;
        std::cout << "[cs_planner]: moveyaw=" << move_angle << ",front_yaw=" << front_yaw << std::endl;
        bool is_backward = (abs(move_angle - front_yaw) > PI / 2) ? true : false;
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

        if (!CollisionCheck(newCSRefPath, mystate, map, avoid_mode)) {
            std::cout << "[cs_planner]: joint result publish" << std::endl;
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
                // std::cout << "[cs_planner]: pub_tmp_x = " << newCSRefPath.r_x[i] << ",tmp_y = " << newCSRefPath.r_y[i] << std::endl;
                // std::cout << "[cs_planner]: pub_tmp_s = " << newCSRefPath.r_s[i] << "[" << i << "]" << std::endl;

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
            _trajectory_pub.publish(trajectory2control);
            trajectory_vis_pub.publish(trajectory_vis);
            UpdateKeeper(trajectory2control.states.back());
            return;
        }
        std::cout << "[cs_planner]: joint failed, generate new one" << std::endl; 
    }

    // version1:regular joint TODO

    // construct CSkeypoint
    double distance = sqrt( pow((target.x - mystate.x), 2) + pow((target.y - mystate.y), 2));
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
        std::cout << "[local_planning] : spline midpoint sample count=" << count << std::endl;
        r_path1.CSclear();
        r_path2.CSclear();
        Spline2D path1(wx1,wy1);
        Spline2D path2(wx2,wy2);
        r_path1.load(path1, overall_length);
        r_path2.load(path2, overall_length);

        if(CollisionCheck(r_path1, mystate, map, avoid_mode)){
            double tmpx1, tmpy1;

            tmpx1 = midpoint(0) + count * 0.1 * abs(cos(perpendicular_angle));
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
            _trajectory_pub.publish(trajectory2control);
            trajectory_vis_pub.publish(trajectory_vis);
            UpdateKeeper(trajectory2control.states.back());
            break;
        }
        if(CollisionCheck(r_path2, mystate, map, avoid_mode)){
            double tmpx2, tmpy2;

            tmpx2 = midpoint(0) - count * 0.1 * abs(cos(perpendicular_angle));
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
            _trajectory_pub.publish(trajectory2control);
            trajectory_vis_pub.publish(trajectory_vis);
            UpdateKeeper(trajectory2control.states.back());
            break;
        }
        ++count;
    }
    std::cout << "[local_planning] : spline_planner failed" << std::endl;
}

void formationCallback(const geometry_msgs::Point::ConstPtr& formation_msg){
    formation_type = formation_msg->x;
    formation_change = true;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"spline_planner");
    ros::NodeHandle nh;

    nh.param<int>("ID", ID, 1);
    nh.param<double>("spline_planner/cubic_spline_hz", cubic_spline_hz, 10.0);
    nh.param<double>("overall_length", overall_length, 0.7);
    nh.param<int>("spline_planner/avoid_mode", avoid_mode, 1);
    nh.param<int>("/formation_type", formation_type, 0);
    nh.param<double>("spline_planner/formation_length_unit", formation_length_unit, 2.5);
    std::cout << "formation_length_unit=" << formation_length_unit << std::endl;
    nh.param<double>("add_lateral_threshold", add_lateral_threshold, 1.5);
    nh.param<double>("add_yaw_threshold", add_yaw_threshold, M_PI / 12);
    nh.param<double>("cs_15_planner/standard_step", standard_step, 0.5);

    ros::Publisher trajectory_pub = nh.advertise<custom_messages::planning_info>("trajectory_cubic_spline", 1, true);
    trajectory_vis_pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_cubic_spline_vis", 1, true);

    CubicSplineClass* myCS = new CubicSplineClass(trajectory_pub);

    ros::Subscriber formation_sub = nh.subscribe("/formationType", 1, formationCallback);
    ros::Subscriber reconstruction_sub = nh.subscribe("/reID", 1, &CubicSplineClass::ReconfigurationCallback, myCS);

    ros::Subscriber platoon0_sub = nh.subscribe("/No_0/myodom", 10, &CubicSplineClass::PlatoonCallback, myCS);
    ros::Subscriber platoon1_sub = nh.subscribe("/No_1/myodom", 10, &CubicSplineClass::PlatoonCallback, myCS);
    ros::Subscriber platoon2_sub = nh.subscribe("/No_2/myodom", 10, &CubicSplineClass::PlatoonCallback, myCS);
    ros::Subscriber platoon3_sub = nh.subscribe("/No_3/myodom", 10, &CubicSplineClass::PlatoonCallback, myCS);
    ros::Subscriber platoon4_sub = nh.subscribe("/No_4/myodom", 10, &CubicSplineClass::PlatoonCallback, myCS);

    ros::Subscriber map_sub = nh.subscribe("slam_ret", 1,  &AstarPathFinder::SlamMapCallback, map);
    ros::Subscriber obs_sub = nh.subscribe("obstacle_info", 1, &CubicSplineClass::ObsCallback, myCS);

    ros::Rate rate(cubic_spline_hz);
    while(ros::ok()){
        ClearpList();
        reconfiguration_flag = 0;

        ros::spinOnce();

        // show pList
        std::cout << "[cs planner]: pList condition" << std::endl;
        for (int i = 0; i < pList.size(); ++i) {
            std::cout << "[" << i << "]:" << pList[i].first << "";
        }
        std::cout << std::endl;

        std::cout << "[cs planner]: node ID = " << ID << std::endl;

        ros::Time time1 = ros::Time::now();
        myCS->planning();
        ros::Time time2 = ros::Time::now();
        std::printf("[cs planner]: Time consume is : %f\n",(time2 - time1).toSec() );

        rate.sleep();
    }
}