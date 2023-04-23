#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "grid_path_searcher/Astar_searcher.h"
//#include "JPS_searcher.h"
// #include "novatel_oem7_msgs/INSPVAX.h"
#include "custom_messages/planning_info.h"
#include "custom_messages/vehicle_status.h"
#include <tf/transform_datatypes.h>

using namespace std;
using namespace Eigen;

// 整车长度，用于分割路径点发布
double overall_length;

double _x, _y, _yaw; // 自身位姿

// ros related
ros::Publisher _grid_path_vis_pub, _grid_map_vis_pub, _path_pub, _dwa_path_vis_pub, _grid_obs_vis_pub;

AstarPathFinder* _astar_path_finder = new AstarPathFinder();
//JPSPathFinder* _jps_path_finder = new JPSPathFinder();

void visGridPath(vector<Vector2d> nodes, bool is_use_jps)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    if(is_use_jps)
        node_vis.ns = "astar_node/jps_path";
    else
        node_vis.ns = "astar_node/astar_path";
    
    node_vis.type = visualization_msgs::Marker::CUBE_LIST; // type表示物体类型，CUBE_LIST表示立方体列表是一系列立方体，除了位姿所有的属性都一样。使用这个物体类型替代 visualization_msgs/MarkerArray允许rviz批处理显示，这让他们的显示更快。附加说明是它们所有都必须有相同的颜色/尺寸。
    node_vis.action = visualization_msgs::Marker::ADD; // 0=add/modify an object, 1=(deprecated), 2=deletes an object, 3=deletes all objects
    node_vis.id = 0; // 分配给marker的唯一的id

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    // 调整颜色
    if(is_use_jps)
    {
        node_vis.color.a = 0.5;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else
    {
        node_vis.color.a = 0.3;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }

    node_vis.scale.x = _astar_path_finder->resolution; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
    node_vis.scale.y = _astar_path_finder->resolution;
    node_vis.scale.z = _astar_path_finder->resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}
void visDWAPath(vector<Vector2d> nodes, bool is_use_jps)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    if(is_use_jps)
        node_vis.ns = "astar_node/jps_path";
    else
        node_vis.ns = "astar_node/dwa_path";
    
    node_vis.type = visualization_msgs::Marker::CUBE_LIST; // type表示物体类型，CUBE_LIST表示立方体列表是一系列立方体，除了位姿所有的属性都一样。使用这个物体类型替代 visualization_msgs/MarkerArray允许rviz批处理显示，这让他们的显示更快。附加说明是它们所有都必须有相同的颜色/尺寸。
    node_vis.action = visualization_msgs::Marker::ADD; // 0=add/modify an object, 1=(deprecated), 2=deletes an object, 3=deletes all objects
    node_vis.id = 0; // 分配给marker的唯一的id

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    // 调整颜色
    if(is_use_jps)
    {
        node_vis.color.a = 0.5;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else
    {
        node_vis.color.a = 0.8;
        node_vis.color.r = 0.5;
        node_vis.color.g = 0.5;
        node_vis.color.b = 0.0;
    }

    node_vis.scale.x = _astar_path_finder->resolution; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
    node_vis.scale.y = _astar_path_finder->resolution;
    node_vis.scale.z = _astar_path_finder->resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        node_vis.points.push_back(pt);
    }

    _dwa_path_vis_pub.publish(node_vis);
}

void visGridObs(vector<Vector2d> nodes){
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "astar_node/obs";
    
    node_vis.type = visualization_msgs::Marker::CUBE_LIST; // type表示物体类型，CUBE_LIST表示立方体列表是一系列立方体，除了位姿所有的属性都一样。使用这个物体类型替代 visualization_msgs/MarkerArray允许rviz批处理显示，这让他们的显示更快。附加说明是它们所有都必须有相同的颜色/尺寸。
    node_vis.action = visualization_msgs::Marker::ADD; // 0=add/modify an object, 1=(deprecated), 2=deletes an object, 3=deletes all objects
    node_vis.id = 0; // 分配给marker的唯一的id

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    // 调整颜色
    node_vis.color.a = 0.5;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _astar_path_finder->resolution; // marker的尺寸，在位姿/方向之前应用，[1,1,1]意思是尺寸是1m*1m*1m
    node_vis.scale.y = _astar_path_finder->resolution;
    node_vis.scale.z = _astar_path_finder->resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        node_vis.points.push_back(pt);
    }

    _grid_obs_vis_pub.publish(node_vis);
}

// void visVisitedNode(vector<Vector2d> nodes)
// {
//     visualization_msgs::Marker node_vis;
//     node_vis.header.frame_id = "map";
//     node_vis.header.stamp = ros::Time::now();
//     node_vis.ns = "astar_node/expanded_nodes";
//     node_vis.type = visualization_msgs::Marker::CUBE_LIST;
//     node_vis.action = visualization_msgs::Marker::ADD;
//     node_vis.id = 0;

//     node_vis.pose.orientation.x = 0.0;
//     node_vis.pose.orientation.y = 0.0;
//     node_vis.pose.orientation.z = 0.0;
//     node_vis.pose.orientation.w = 1.0;
//     node_vis.color.a = 0.5;
//     node_vis.color.r = 0.0;
//     node_vis.color.g = 0.0;
//     node_vis.color.b = 1.0;

//     node_vis.scale.x = _astar_path_finder->resolution;
//     node_vis.scale.y = _astar_path_finder->resolution;
//     node_vis.scale.z = _astar_path_finder->resolution;

//     geometry_msgs::Point pt;
//     for(int i = 0; i < int(nodes.size()); i++)
//     {
//         Vector2d coord = nodes[i];
//         pt.x = coord(0);
//         pt.y = coord(1);

//         node_vis.points.emplace_back(pt);
//     }

//     _visited_nodes_vis_pub.publish(node_vis);
// }

void pathFinding(const Vector2d start_pt, const Vector2d target_pt)
{
    // 调用A*算法来搜索路径
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
    // Retrieve path
    auto grid_path = _astar_path_finder->getPath();

    // vector<Vector2d> dwa_path;
    Vector2d tmp2d;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped point;
    for(int i = 0; i < grid_path.size(); i++){
        point.pose.position.x = grid_path[i](0);
        point.pose.position.y = grid_path[i](1);
        path.poses.push_back(point);

        // tmp2d<<grid_path[i](0),grid_path[i](1);
        // dwa_path.emplace_back(tmp2d);
        cout<< "dwa path : " << point.pose.position.x << ", "<<point.pose.position.y<<endl;
    }

    _path_pub.publish(path);

    // visualize
    visGridPath(grid_path, false);
    // visDWAPath(dwa_path,false);

    // Reset map for next call
    _astar_path_finder->resetUsedGrids();


//_use_jps = 0 -> Do not use JPS
//_use_jps = 1 -> Use JPS
//you just need to change the #define value of _use_jps
#define _use_jps 0
#if _use_jps
    {
        //Call JPS to search for a path
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, _use_jps);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
    }
#endif
}

// For autoinit
// For GPS
// void LocationCallback(const custom_messages::planning_info::ConstPtr& self_loc_msg)
// {
//     _x = self_loc_msg->states[1].xPos;
//     _y = self_loc_msg->states[1].yPos;
//     _yaw = self_loc_msg->states[1].yaw;
//     myinit_x = self_loc_msg->states[0].xPos;
//     myinit_y = self_loc_msg->states[0].yPos;
//     myinit_yaw = self_loc_msg->states[0].yaw;
// }
// For odom
// void LocationCallback(const nav_msgs::Odometry::ConstPtr& self_loc_msg)
// {
//     _x = self_loc_msg->pose.pose.position.x;
//     _y = self_loc_msg->pose.pose.position.y;
//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(self_loc_msg->pose.pose.orientation, quat);
//     double roll, pitch, yaw;//定义存储r\p\y的容器
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
//     _yaw = yaw;
//     if(astar_first){
//         myinit_x = _x;
//         myinit_y = _y;
//         myinit_yaw = yaw;
//     }
// }
// For humaninit
void LocationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_loc_msg){
    _x = init_loc_msg->pose.pose.position.x;
    _y = init_loc_msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(init_loc_msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    _yaw = yaw;
    ROS_INFO("[routing] : A* initial, x : %f, y : %f, yaw : %f", _x, _y, _yaw);
}

void AstarPathFinder::rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& wp)
{
    // astar_first = true;
    // 将终点信息传递给终点指针
    Vector2d target_pt;
    target_pt << wp->pose.position.x, wp->pose.position.y;

    ROS_WARN("[routing] : A* received target : %f , %f", target_pt(0), target_pt(1));

    Vector2d start_pt;
    start_pt << _x, _y;
    // 调用路径规划函数
    pathFinding(start_pt, target_pt);
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
    _astar_path_finder->inflation();
	// ofstream out2("/home/csxr/out2.txt", ios::app);
    // for (int j = 0; j <_slam_map.data.size(); ++j) {
    //     out2<<(int)_slam_map.data[j]<<" ";
    // }
    initGridMap();

    visited_node_vis.header.frame_id = "map";
    visited_node_vis.header.stamp = ros::Time::now();
    visited_node_vis.ns = "astar_node/expanded_nodes";
    visited_node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    visited_node_vis.action = visualization_msgs::Marker::ADD;
    visited_node_vis.id = 0;
    visited_node_vis.pose.orientation.x = 0.0;
    visited_node_vis.pose.orientation.y = 0.0;
    visited_node_vis.pose.orientation.z = 0.0;
    visited_node_vis.pose.orientation.w = 1.0;
    visited_node_vis.color.a = 0.5;
    visited_node_vis.color.r = 0.0;
    visited_node_vis.color.g = 0.0;
    visited_node_vis.color.b = 1.0;
    visited_node_vis.scale.x = _astar_path_finder->resolution;
    visited_node_vis.scale.y = _astar_path_finder->resolution;
    visited_node_vis.scale.z = _astar_path_finder->resolution;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "A*_node");
    ros::NodeHandle nh;

    // ros::Subscriber mypoint = nh.subscribe("self_location", 2, LocationCallback);
    ros::Subscriber mypoint = nh.subscribe("initialpose", 2, LocationCallback);
    ros::Subscriber _map_sub = nh.subscribe("slam_ret", 1,  &AstarPathFinder::SlamMapCallback, _astar_path_finder);
    ros::Subscriber _pts_sub = nh.subscribe("waypoints", 1, &AstarPathFinder::rcvGoalCallback, _astar_path_finder);

    // _grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _astar_path_finder->_visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1);
    _path_pub = nh.advertise<nav_msgs::Path>("routing_path", 1);
    _grid_obs_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_obs_vis", 1);
    // _dwa_path_vis_pub = nh.advertise<visualization_msgs::Marker>("dwa_path_vis", 1);

    nh.param<double>("overall_length", overall_length, 1.0);

    //    _jps_path_finder    = new JPSPathFinder();
    //    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    ros::Rate rate(10);
    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();
        vector<Vector2d> grid_obs;
        for(int i = 0; i <_astar_path_finder->_slam_map.data.size(); ++i){
            if(_astar_path_finder->_slam_map.data[i] == 100){
                int tx = i % _astar_path_finder->GLX_SIZE;
                int ty = i / _astar_path_finder->GLX_SIZE;
                Vector2d tmp;
                tmp << _astar_path_finder->gl_xl + tx * _astar_path_finder->resolution, _astar_path_finder->gl_yl + ty * _astar_path_finder->resolution;
                grid_obs.emplace_back(tmp);
            }
        }
        visGridObs(grid_obs);
        status = ros::ok();
        rate.sleep();
    }
    // ros::spin();

    return 0;
}