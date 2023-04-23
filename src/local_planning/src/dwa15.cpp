#include "dwa15.h"

double max_speed;
double min_speed;
double max_angular_speed;
double min_angular_speed;

double max_accel;
double max_angular_speed_rate;

int v_resolution;     // 速度采样分辨率
int yaw_rate_resolution;
double dt;                //运动学模型预测时间
double predict_time;
double goal_cost_gain;
double speed_cost_gain;
double overall_length;
int ID;
double dwa_hz;
int avoid_mode;
int formation_type;

ros::Publisher trajectory_vis_pub, DWA_path_point_pub;
geometry_msgs::PoseArray trajectory_vis;

AstarPathFinder* dwamap = new AstarPathFinder();
pcl::PointCloud<pcl::PointXY>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXY>);
pcl::search::KdTree<pcl::PointXY>::Ptr tree (new pcl::search::KdTree<pcl::PointXY>);

void visDWAPath(vector<Eigen::Vector2d> nodes){
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "DWA/path";
    
    node_vis.type = visualization_msgs::Marker::CUBE_LIST; // type表示物体类型，CUBE_LIST表示立方体列表是一系列立方体，除了位姿所有的属性都一样。使用这个物体类型替代 visualization_msgs/MarkerArray允许rviz批处理显示，这让他们的显示更快。附加说明是它们所有都必须有相同的颜色/尺寸。
    node_vis.action = visualization_msgs::Marker::ADD; // 0=add/modify an object, 1=(deprecated), 2=deletes an object, 3=deletes all objects
    node_vis.id = 0; // 分配给marker的唯一的id

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    // 调整颜色
    node_vis.color.a = 0.7;
    node_vis.color.r = 0.5;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.5;

    node_vis.scale.x = 0.1; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.1;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        node_vis.points.push_back(pt);
    }

    DWA_path_point_pub.publish(node_vis);
}

DWA::DWA(ros::Publisher pub){
    trajectory_pub = pub;
}

void DWA::PathCallback(const nav_msgs::Path::ConstPtr& path_msg){
    ends = *path_msg;
    iteration = 0;
    myend.x = ends.poses.front().pose.position.x;
    myend.y = ends.poses.front().pose.position.y;
}

void DWA::Status15toCarstate(const custom_messages::status15::ConstPtr& input, CarState& output) {
    output.x = input->position.x;
    output.y = input->position.y;
    output.yaw = input->pose.yaw;
    output.speed = input->linear_velocity.x;
    output.angular_speed = input->angular_velocity.z;
}

// void DWA::SelfCallback(const nav_msgs::Odometry::ConstPtr& self_msg){
//     mystate.x = self_msg->pose.pose.position.x;
//     mystate.y = self_msg->pose.pose.position.y;
//     tf::Quaternion RQ2;
//     double roll,pitch,yaw;
//     tf::quaternionMsgToTF(self_msg->pose.pose.orientation,RQ2);  
//     tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);  
//     mystate.yaw = yaw;
//     mystate.speed = self_msg->twist.twist.linear.x;
//     mystate.angular_speed = self_msg->twist.twist.angular.z;
// }
void DWA::SelfCallback(const custom_messages::status15::ConstPtr& self_msg) {
    Status15toCarstate(self_msg, mystate);
}

inline bool CheckDWAArrive(CarState now, CarState end){
    if(sqrt(pow(now.x - end.x, 2) + pow(now.y - end.y, 2)) <= 2*overall_length) return true;
    else return false;
}
inline bool CheckDWAArrive(CarState now, CarState end, double FLcoef){
    if(sqrt(pow(now.x - end.x, 2) + pow(now.y - end.y, 2)) <= FLcoef*overall_length) return true;
    else return false;
}

void DWA::findMatchPoint(const CarState &now_pos, const nav_msgs::Path refPath){
    double mindis = 1 << 20;
    for(int i = iteration; i < refPath.poses.size(); ++i){
        double tmpdis = pow(now_pos.x - refPath.poses[i].pose.position.x, 2) + pow(now_pos.y - refPath.poses[i].pose.position.y, 2);
        if(tmpdis < mindis){
            mindis = tmpdis;
            iteration = i;
        }
    }
    if(iteration < refPath.poses.size()){
        myend.x = ends.poses[iteration].pose.position.x;
        myend.y = ends.poses[iteration].pose.position.y;
    }
}

void DWA::planning()
{
    if(ends.poses.empty()){
        ROS_WARN("[local_planning] : no routing path");
        return;
    } 
    CarState currentState(mystate.x, mystate.y, mystate.yaw, mystate.speed, mystate.angular_speed);
    std::cout << "mystate.x" << mystate.x << "mystate.y" << mystate.y << "mystate.speed" << mystate.speed << "mystate.aspeed" << mystate.angular_speed << std::endl;

    // cout<<"currentState.x : "<<currentState.x<<"currentState.y : "<<currentState.y<<endl;
    // cout<<"myend.x : "<<myend.x<<"myend.y : "<<myend.y<<endl;
    // cout<<"sqrt : "<<sqrt(pow(currentState.x - myend.x, 2) + pow(currentState.y - myend.y, 2))<< ", tol : " << 2.5*overall_length <<endl;
    
    findMatchPoint(currentState, ends);
    while(CheckDWAArrive(currentState, myend)){
        if(++iteration < ends.poses.size()){
            myend.x = ends.poses[iteration].pose.position.x;
            myend.y = ends.poses[iteration].pose.position.y;
            cout<< "[local_planning] : Going to next end : " <<myend.x<<" , "<<myend.y<<endl;
        }
        else{
            ROS_WARN("[local_planning] : DWA Done!");
            custom_messages::planning_info trajectory2control;
            trajectory_pub.publish(trajectory2control);
            return;
        }
    }

    std::cout << "[dwa]: generate new dwa trajectory" << std::endl;
    vector<Eigen::Vector2d> DWA_path_point_vis;
    Eigen::Vector2d pt;
    pt<< myend.x, myend.y;
    DWA_path_point_vis.emplace_back(pt);

    vector<CarState> currentTrajectory;

    vector<double> speed(2);     //v[0]为速度, v[1]角速度
    speed = dwa_control(currentState);
    // cout << "final speed:(" << speed[0] << ", " << speed[1] << ")" << endl;
    currentTrajectory.clear();
    predict_trajectory(currentState, speed[0], speed[1], currentTrajectory);
    trajectory.push_back(currentTrajectory);

    trajectory_vis.poses.clear();
    trajectory_vis.header.frame_id = "map";
    trajectory_vis.header.stamp = ros::Time::now();

    custom_messages::planning_info trajectory2control;
    custom_messages::vehicle_status tmp;
    geometry_msgs::Pose tmppose;
    for(int i = 0; i < currentTrajectory.size(); ++i){
        tmp.xPos = currentTrajectory[i].x;
        tmp.yPos = currentTrajectory[i].y;
        tmp.yaw = currentTrajectory[i].yaw;
        tmp.speed = currentTrajectory[i].speed;
        tmp.angular_speed = currentTrajectory[i].angular_speed;
        trajectory2control.states.push_back(tmp);

        tmppose.position.x = currentTrajectory[i].x;
        tmppose.position.y = currentTrajectory[i].y;
        double tmpyaw = currentTrajectory[i].yaw;
        //输入欧拉角，转化成四元数在终端输出
        geometry_msgs::Quaternion q;
        q=tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpyaw);
        tmppose.orientation.w = q.w;
        tmppose.orientation.x = q.x;
        tmppose.orientation.y = q.y;
        tmppose.orientation.z = q.z;
        trajectory_vis.poses.emplace_back(tmppose);
    }
    trajectory_vis_pub.publish(trajectory_vis);
    trajectory_pub.publish(trajectory2control);
    visDWAPath(DWA_path_point_vis);
}

//动态窗口法
vector<double> DWA::dwa_control(const CarState &carstate)
{
    vector<double> dw(4);     //dw[0]为最小速度，dw[1]为最大速度，dw[2]为最小角速度，dw[3]为最大角速度
    //计算动态窗口
    dw = calc_dw(carstate);
    // for(int i = 0; i < dw.size(); ++i){
    //     cout<<"dw[i] : "<<dw[i]<<endl;
    // }
    //计算最佳（v, w）
    vector<double> v_w(2);
    v_w = calc_best_speed(carstate, dw);
    return v_w;
}
// 计算动态窗口
vector<double> DWA::calc_dw(const CarState &carstate)
{
    // 机器人速度属性限制的动态窗口
    vector<double> dw_robot_state{min_speed, max_speed, min_angular_speed, max_angular_speed};
    // 机器人模型限制的动态窗口
    vector<double> dw_robot_model(4);
    dw_robot_model[0] = carstate.speed - max_accel;
    dw_robot_model[1] = carstate.speed + max_accel;
    dw_robot_model[2] = carstate.angular_speed - max_angular_speed_rate;
    dw_robot_model[3] = carstate.angular_speed + max_angular_speed_rate;
    // cout<<"dw_robot_model  "<< dw_robot_model[0] << ", "<< dw_robot_model[1] << ", "<< dw_robot_model[2] << ", "<< dw_robot_model[3] <<endl;
    vector<double> dw{max(dw_robot_state[0], dw_robot_model[0]),
                     min(dw_robot_state[1], dw_robot_model[1]),
                     max(dw_robot_state[2], dw_robot_model[2]),
                     min(dw_robot_state[3], dw_robot_model[3])};
    return dw;
}
//在dw中计算最佳速度和角速度
vector<double> DWA::calc_best_speed(const CarState &carstate, const vector<double> &dw)
{
    vector<double> best_speed{0, 0};
    vector<CarState> trajectoryTmp;
    double min_cost = 1<<20;
    double final_cost;
    double goal_cost;
    double speed_cost = 0;
    double delta_v = (dw[1] - dw[0]) / v_resolution;
    double delta_w = (dw[3] - dw[2]) / yaw_rate_resolution;
    for(double i = dw[0]; i < dw[1]; i += delta_v)
    {
        for (double j = dw[2]; j < dw[3]; j += delta_w)
        {
            //预测轨迹
            trajectoryTmp.clear();
            predict_trajectory(carstate, i, j, trajectoryTmp);
            //计算代价
            goal_cost = goal_cost_gain * calc_goal_cost(trajectoryTmp) / PI;
            speed_cost = speed_cost_gain * (dw[1] - trajectoryTmp.back().speed) / dw[1];            // speed_cost = speed_cost_gain * (max_speed - trajectoryTmp.back().speed);
            if(CollisionCheck(trajectoryTmp, carstate, dwamap, avoid_mode, ID)){
                continue;
            }
            final_cost = goal_cost + speed_cost;

            if(final_cost < min_cost)
            {
                min_cost = final_cost;
                best_speed[0] = i;
                best_speed[1] = j;
                // cout << "tmp_best_speed : " <<best_speed[0]<<", "<<best_speed[1]<<endl;
            }
        }
    }
    return best_speed;
}
// 在一段时间内预测轨迹
void DWA::predict_trajectory(const CarState &carstate, const double &speed, const double &angular_speed, vector<CarState> &trajectory)
{
    double time = 0;
    CarState nextState = carstate;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    while(time < predict_time)
    {
        nextState = motion_model(nextState, speed, angular_speed);
        // cout << "nextState:(" << nextState.x << ", " << nextState.y << ", " << nextState.yaw * 180 / PI << ")" << nextState.speed << "  " << nextState.angular_speed << endl;
        trajectory.push_back(nextState);
        time += dt;
    }
}
//根据动力学模型计算下一时刻状态
CarState DWA::motion_model(const CarState &carstate, const double &speed, const double &angular_speed)
{
    CarState nextState;
    nextState.x = carstate.x + speed * dt * cos(carstate.yaw);
    nextState.y = carstate.y + speed * dt * sin(carstate.yaw);
    nextState.yaw = carstate.yaw + angular_speed * dt;
    // std::cout << "nextStateYaw : " <<nextState.yaw << std::endl;
    nextState.speed = carstate.speed;
    nextState.angular_speed = carstate.angular_speed;
    return nextState;
}
// 计算方位角代价
double DWA::calc_goal_cost(const vector<CarState> &trajectory)
{
    double error_yaw = atan2(myend.y - trajectory.back().y, myend.x - trajectory.back().x);
    double goal_cost = error_yaw - trajectory.back().yaw;
    goal_cost = atan2(sin(goal_cost), cos(goal_cost));
    if(goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
}

bool DWA::CollisionCheck(const vector<CarState> &trajectory, const CarState &carstate, const AstarPathFinder* tmpmap, int mode, int id)
{
    double distance;
    if(mode){
        for (int i = 0; i < mymapmodel.mapinfo.size(); i ++) {
            if(id){
                double FLtarget2obs = sqrt(pow(mymapmodel.mapinfo[i].x - myend.x, 2) + pow(mymapmodel.mapinfo[i].y - myend.y, 2));
                if(FLtarget2obs <= mymapmodel.mapinfo[i].z) continue;
            }
            for (int j = 0; j < trajectory.size(); j ++) {
                distance = sqrt(pow(mymapmodel.mapinfo[i].x - trajectory[j].x, 2) + pow(mymapmodel.mapinfo[i].y - trajectory[j].y, 2));
                if(distance <= mymapmodel.mapinfo[i].z + overall_length)
                    return true;
            }
        }
    }

    double window = predict_time * carstate.speed + 0.5 * max_accel * pow(predict_time, 2);
    // pcl
    // cout << "map size: " << mapCloud->size() << endl;
    // cout << mapCloud->points[0].x << mapCloud->points[0].y << endl;
    // pcl::Indices k_indices;
    vector<int> k_indices;
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
            for (int k = 0; k < trajectory.size(); k++) {
                distance = sqrt(pow(p.x - trajectory[k].x, 2) + pow(p.y - trajectory[k].y, 2));
                if(distance <= overall_length/1.5)
                    return true;
            }     
        }
    }
    
    return false;
}

// 加载感知
void DWA::MapModelCallback(const custom_messages::mapmodel::ConstPtr& mapmodel_msg){
    if(mapmodel_msg->mapinfo.empty()){
        // cout << "[local planning] : all clear" << endl;
        return;
    }
    mymapmodel = *mapmodel_msg;
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
    dwamap->inflation();
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

int main(int argc, char** argv){
    ros::init(argc, argv, "DWA");
    ros::NodeHandle n;

    n.param<int>("ID", ID, 0);
    n.param<double>("vmax", max_speed, 1.0);
    n.param<double>("vmin", min_speed, -0.5);
    n.param<double>("wmax", max_angular_speed, 40 * PI / 180.0);
    n.param<double>("wmin", min_angular_speed, -40 * PI / 180.0);
    n.param<double>("dwa/dv", max_accel, 0.2);
    n.param<double>("dwa/dw", max_angular_speed_rate, 20 * PI / 180);
    n.param<int>("v_resolution", v_resolution, 10);
    n.param<int>("yaw_rate_resolution", yaw_rate_resolution, 10);
    n.param<double>("dt", dt, 0.1);
    n.param<double>("predict_time", predict_time, 2.0);
    n.param<double>("goal_cost_gain", goal_cost_gain, 0.2);
    n.param<double>("speed_cost_gain", speed_cost_gain, 1.0);
    n.param<double>("overall_length", overall_length, 0.7);
    n.param<double>("dwa/dwa_hz", dwa_hz, 1.0);
    n.param<int>("dwa/avoid_mode", avoid_mode, 1);
    n.param<int>("formation_type", formation_type, 0);

    ros::Publisher trajectory_pub = n.advertise<custom_messages::planning_info>("trajectory_dwa", 1);
    trajectory_vis_pub = n.advertise<geometry_msgs::PoseArray>("trajectory_dwa_vis", 1, true);
    DWA_path_point_pub = n.advertise<visualization_msgs::Marker>("dwa_path_point_vis", 1, true);

    DWA* mydwa = new DWA(trajectory_pub);

    ros::Subscriber dwa_map_sub = n.subscribe("slam_ret", 1,  &AstarPathFinder::SlamMapCallback, dwamap);
    ros::Subscriber mapmodel_sub = n.subscribe("obstacle_info", 1, &DWA::MapModelCallback, mydwa);
    ros::Subscriber self_sub = n.subscribe("self_location", 1, &DWA::SelfCallback, mydwa);
    ros::Subscriber path_sub = n.subscribe("routing_path", 1, &DWA::PathCallback, mydwa);

    ros::Rate rate(dwa_hz);
    while(ros::ok()){
        ros::spinOnce();
        // ros::Time time1 = ros::Time::now();
        mydwa->planning();
        // ros::Time time2 = ros::Time::now();
        // ROS_INFO("[local planning] : DWA Time consume is : %f",(time2 - time1).toSec() );
        rate.sleep();
    }
}