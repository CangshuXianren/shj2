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

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include "rrt_searcher/rrtstar_searcher.h"

using namespace std;
using namespace Eigen;

// 在以下的ompl库使用中存在着很多模板的使用，建议同时学习ompl的模板
namespace ob = ompl::base;
namespace og = ompl::geometric;

//launch文件参数
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size;    

bool _has_map = false;

Vector2d _start_pt;
Vector2d _map_lower, _map_upper;
int _max_x_id, _max_y_id;


// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_map_vis_pub, _RRTstar_path_vis_pub;

RRTstarPreparatory * _RRTstar_preparatory     = new RRTstarPreparatory();

void pathFinding(const Vector2d start_pt, const Vector2d target_pt);
void rcvWaypointsCallback(const nav_msgs::Path::ConstPtr& wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_map);
void visRRTstarPath(vector<Vector2d> nodes );

void rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& wp)
{
    if( _has_map == false) return;
    Vector2d target_pt;
    target_pt << wp->pose.position.x, wp->pose.position.y;
    ROS_INFO("[RRT*_node] : received planning target!");
    pathFinding(_start_pt, target_pt);
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_map)
{   
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(*pointcloud_map, cloud);
    
    if( cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        // set obstalces into grid map for path planning
        _RRTstar_preparatory->setObs(pt.x, pt.y);

        // for visualize only
        Vector2d cor_round = _RRTstar_preparatory->coordRounding(Vector2d(pt.x, pt.y));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "map";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

//碰撞检测
class ValidityChecker : public ob::StateValidityChecker 
{
public:
    // 自定义状态评估器
    ValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si) {}
    //检查给定的状态所处位置是否位于障碍物上
    bool isValid(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* nowstate = state->as<ob::RealVectorStateSpace::StateType>();

        //从机器人的state获取其(x,y)
        auto x = (*nowstate)[0];
        auto y = (*nowstate)[1];

        return _RRTstar_preparatory->checkstate(x, y);
    }
};

// 碰撞检测
class myMotionValidator : public ob::DiscreteMotionValidator
{
public:
    myMotionValidator(const ob::SpaceInformationPtr& si) : ob::DiscreteMotionValidator(si) {}

    bool checkMotion(const ob::State *s1, const ob::State *s2) const
    {
         const ob::RealVectorStateSpace::StateType* nowstate1 = s1->as<ob::RealVectorStateSpace::StateType>();
         const ob::RealVectorStateSpace::StateType* nowstate2 = s2->as<ob::RealVectorStateSpace::StateType>();

        //从机器人的state获取其(x,y)
        auto x1 = (*nowstate1)[0];
        auto y1 = (*nowstate1)[1];
        auto x2 = (*nowstate2)[0];
        auto y2 = (*nowstate2)[1];
        return _RRTstar_preparatory->isObsFree(x1, y1, x2, y2);
    }
};

// 定义优化目标为最小化路径
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

// 定义优化阈值
ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

void pathFinding(const Vector2d start_pt, const Vector2d target_pt)
{
    // ompl使用一般步骤:
    //     1.实例化一个状态空间;
    //     2.实例化一个控制空间(可选);
    //     3.实例化一个空间信息类;
    //     4.实例化一个问题描述;
    //     5.实例化一个规划器;
    //     6.路径简化器.

    // 构造状态空间
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // 设置空间边界
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, - _x_size * 0.5);
    bounds.setLow(1, - _y_size * 0.5);
    bounds.setHigh(0, _x_size * 0.5);
    bounds.setHigh(1, _y_size * 0.5);

    // 将设置的边界赋予状态空间space，as函数功能为类型转换
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // 构造状态空间信息SpaceInformation
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // 为状态空间设置状态检查函数StateValidityChecker
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setMotionValidator(ob::MotionValidatorPtr(new myMotionValidator(si)));

    // 载入设置
    si->setup();

    // 设置起点
    ob::ScopedState<> start(space);
        // 方法1:
    // start[0] = (&start_pt)->operator[](0)  ;
    // start[1] = (&start_pt)->operator[](1)  ;
    // start[2] = (&start_pt)->operator[](2)  ;
    // 方法2:
    // start[0] = start_pt(0);
    // start[1] = start_pt(1);
    // start[2] = start_pt(2);
    // 方法3:
    // start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_pt(0);
    // start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_pt(1);
    // start->as<ob::RealVectorStateSpace::StateType>()->values[2] = start_pt(2);
    // 方法4:
    start->as<ob::RealVectorStateSpace::StateType>()->operator[](0) = start_pt(0);
    start->as<ob::RealVectorStateSpace::StateType>()->operator[](1) = start_pt(1);

    // 设定终点
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->operator[](0) = target_pt(0);
    goal->as<ob::RealVectorStateSpace::StateType>()->operator[](1) = target_pt(1);

    // 构造问题实例，传入之前构造的状态空间信息SpaceInformation
    auto pdef(make_shared<ob::ProblemDefinition>(si));

    // 为问题实例设置起点和终点
    pdef->setStartAndGoalStates(start, goal);

    // 构造优化器实例，这里使用的是之前定义的长度优化器
    // 方法1:
    // pdef->setOptimizationObjective(getPathLengthObjective(si));
    // 方法2:
    pdef->setOptimizationObjective(make_shared<ob::PathLengthOptimizationObjective>(si));

    // 构造RRT*规划器
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

    // 传入问题实例给规划器
    optimizingPlanner->setProblemDefinition(pdef);

    // 载入设置
    optimizingPlanner->setup();

    // 以1s的时间求解规划问题
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

    //如果有解就将得到的路径压入堆栈，将路径在rviz中显示
    if(solved)
    {
        // 从问题定义中获取目标信息（不是终点状态）并询问寻找到的路径
        og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();

        vector<Vector2d> path_points;

        for(size_t path_idx = 0;  path_idx < path->getStateCount(); ++path_idx)
        {
            const ob::RealVectorStateSpace::StateType* state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>();

            // 可视化
            // 方法1:
            // auto x = state->operator[](0);
            // auto y = state->operator[](1);
            // auto z = state->operator[](2);
            // 方法2:
            // auto x = (*state)[0];
            // auto y = (*state)[1];
            // auto z = (*state)[2];
            // 方法3:
            auto x = state->values[0];
            auto y = state->values[1];
            Vector2d temp_mat(x,y);
            path_points.push_back(temp_mat);
        }
        visRRTstarPath(path_points);    
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RRT*_node");
    ros::NodeHandle nh("~");

    _map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);
    _pts_sub = nh.subscribe("waypoints", 1, rcvGoalCallback);

    _grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _RRTstar_path_vis_pub = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis", 1);

    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);

    _RRTstar_preparatory  = new RRTstarPreparatory();
    _RRTstar_preparatory  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _RRTstar_preparatory;
    return 0;
}

void visRRTstarPath(vector<Vector2d> nodes )
{
    visualization_msgs::Marker Points, Line; 
    Points.header.frame_id = Line.header.frame_id = "map";
    Points.header.stamp    = Line.header.stamp    = ros::Time::now();
    Points.ns              = Line.ns              = "RRT*_node/RRTstarPath";
    Points.action          = Line.action          = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id   = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = _resolution/4; 
    Points.scale.y = _resolution/4;
    Line.scale.x   = _resolution/4;

    //points are green and Line Strip is blue
    Points.color.g = 1.0;
    Points.color.a = 1.0;
    Line.color.b   = 1.0;
    Line.color.a   = 1.0;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector2d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line); 
}