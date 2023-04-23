#ifndef _ASTAR_SEARCHER_H
#define _ASTAR_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "astarnode.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class AstarPathFinder
{
public:
    //地图指针
    GridNodePtr** GridNodeMap;
    //网格地图坐标形式的目标点
    Eigen::Vector2i goalIdx;

    //终点指针
    GridNodePtr terminatePtr;
    //openSet容器，用于存放规划中已确定的路径点
    std::multimap<double, GridNodePtr> openSet;

    //启发函数，返回两点之间的距离(曼哈顿距离/欧式距离/对角线距离)
    double getHeu(GridNodePtr node1, GridNodePtr node2);

    /*
     * currentPtr   当前传入点的指针
     * edgeCostSets currentPtr到某个邻居的实际坐标距离
     * neighborPtrSets  邻居集
     */
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr>& neighborPtrSets, std::vector<double>& edgeCostSets);

    //判断结点是否为障碍物(data对应的元素为1则为障碍物)
    bool isOccupied(const int& idx_x, const int& idx_y) const;
    bool isOccupied(const Eigen::Vector2i& index) const;
    //判断结点是否为空(data对应的元素为0则为空)
    bool isFree(const int& idx_x, const int& idx_y) const;
    bool isFree(const Eigen::Vector2i& index) const;

    //栅格地图坐标转实际坐标
    Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i& index);
    //实际坐标转栅格坐标
    Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d& pt);
    

    AstarPathFinder();
    ~AstarPathFinder();

    // 终点信息的回调函数
    void rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& wp);
    // 地图信息的回调函数
    void SlamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& slam_map_msgs);
    // 自己保存的地图信息
    nav_msgs::OccupancyGrid _slam_map;
    //栅格地图的长宽高
    int GLX_SIZE, GLY_SIZE, GLXY_SIZE;
    //resolution表示栅格地图精度(即一格几米)，inv_resolution=1/resolution
    double resolution, inv_resolution;
    //gl表示实际坐标中的地图边界，l表示下边界，u表示上边界
    double gl_xl, gl_yl;
    double gl_xu, gl_yu;

    ros::Publisher _visited_nodes_vis_pub;
    visualization_msgs::Marker visited_node_vis;

    //A*路径搜索
    bool AstarGraphSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);
    //将所有点的属性设置为未访问的状态
    void resetGrid(GridNodePtr ptr);
    //通过循环遍历重置每一个点
    void resetUsedGrids();
    //初始化地图
    void initGridMap();
    //地图膨胀
    void inflation();

    Eigen::Vector2d coordRounding(const Eigen::Vector2d& coord);
    
    //获取A*搜索得到的完整路径
    std::vector<Eigen::Vector2d> getPath();
    //获得访问过的所有结点
    std::vector<Eigen::Vector2d> getVisitedNodes();
};

#endif