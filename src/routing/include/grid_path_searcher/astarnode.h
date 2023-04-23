#ifndef _ASTARNODE_H_
#define _ASTARNODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <fstream>

#define inf 1<<20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
    //1为open set，-1为closed set，0为未访问
    int id;
    //实际坐标
    Eigen::Vector2d coord;
    //direction of expanding
    Eigen::Vector2i dir;
    //栅格地图坐标
    Eigen::Vector2i index;

    double gScore, hScore, fScore;

    //父结点
    GridNodePtr cameFrom;

    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector2i _index, Eigen::Vector2d _coord)
    {
        id = 0;
        index = _index;
        coord = _coord;
        dir = Eigen::Vector2i::Zero();

        gScore = inf;
        hScore = inf;
        fScore = inf;
        cameFrom = nullptr;
    }

    ~GridNode();
};

#endif