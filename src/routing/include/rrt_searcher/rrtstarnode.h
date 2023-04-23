#ifndef _RRTSTARNODE_H_
#define _RRTSTARNODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>

#define inf 1<<20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(){};
    ~GridNode(){};
};


#endif
