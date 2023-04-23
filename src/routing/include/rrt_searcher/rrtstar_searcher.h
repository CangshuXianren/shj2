#ifndef _RRTSTAR_SEARCHER_H_
#define _RRTSTAR_SEARCHER_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "rrtstarnode.h"

class RRTstarPreparatory
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr ** GridNodeMap;

		int GLX_SIZE, GLY_SIZE, GLXY_SIZE;

		double resolution, inv_resolution;
		double gl_xl, gl_yl;
		double gl_xu, gl_yu;	

		Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i & index);
		Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d & pt);

	public:
		RRTstarPreparatory(){};
		~RRTstarPreparatory(){};

		void initGridMap(double _resolution, Eigen::Vector2d global_xy_l, Eigen::Vector2d global_xy_u, int max_x_id, int max_y_id);
		void setObs(const double coord_x, const double coord_y);
		bool checkstate(const double coord_x1, const double coord_y1);
		bool isObsFree(const double coord_x1, const double coord_y1, const double coord_x2, const double coord_y2);
		
		Eigen::Vector2d coordRounding(const Eigen::Vector2d & coord);
};

#endif