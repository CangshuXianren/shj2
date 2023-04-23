// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <cmath>

#include <Twist.h>
#include <math_utils.h>
#include <pcl/point_types.h>
//#include <libpcan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/io.h>

/*
#define DEFAULT_NODE "/dev/pcanpci1"
HANDLE pcan_handle =NULL;//void *pcan_handle
*/

typedef pcl::PointXYZI PointType;

template <typename PointT>
void publishCloudMsg(ros::Publisher& publisher,
			    const pcl::PointCloud<PointT>& cloud,
			    std::string frameID)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frameID;
    publisher.publish(msg);
}

template <typename PointT>
void publishPointsVisualMsg(ros::Publisher& publisher,
				  const pcl::PointCloud<PointT>& cloud,
				  float r,
				  float g,
				  float b,
				  std::string frameID)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cone";
    marker.id = 3;

    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.3;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for(size_t i = 0; i < cloud.points.size(); i ++)
    {
	geometry_msgs::Point p_v;
	p_v.x = cloud.points[i].x;
	p_v.y = cloud.points[i].y;
	p_v.z = 0;

	marker.points.push_back(p_v);
    }
    publisher.publish(marker);
}

template <typename PointT>
void publishObjVisualMsg(ros::Publisher& publisher,
				  const pcl::PointCloud<PointT>& cloud,
				  float r, float g, float b,
				  float sizex,
				  float sizey,
				  std::string frameID)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cone";
    marker.id = 3;

    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = sizex;
    marker.scale.y = sizey;
    marker.scale.z = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for(size_t i = 0; i < cloud.points.size(); i ++)
    {
	geometry_msgs::Point p_v;
	p_v.x = cloud.points[i].x;
	p_v.y = cloud.points[i].y;
	p_v.z = 0;

	marker.points.push_back(p_v);
    }
    publisher.publish(marker);
}

template <typename PointT>
void publishPointsVisualMsg(ros::Publisher& publisher,
				  const std::vector<PointT>& cloud,
				  float r,
				  float g,
				  float b,
				  std::string frameID)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cone";
    marker.id = 3;

    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.3;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for(size_t i = 0; i < cloud.size(); i ++)
    {
	geometry_msgs::Point p_v;
	p_v.x = cloud[i].x;
	p_v.y = cloud[i].y;
	p_v.z = 0;

	marker.points.push_back(p_v);
    }

    publisher.publish(marker);
}

template <typename PointT>
void publishLineVisualMsg(ros::Publisher& publisher,
			    const std::vector<PointT>& cloud,
			    std::string frameID)
{
    visualization_msgs::Marker linemarker;

    linemarker.header.frame_id = frameID;
    linemarker.header.stamp = ros::Time::now();
    linemarker.ns = "line";
    linemarker.action = visualization_msgs::Marker::ADD;
    linemarker.pose.orientation.w = 1.0;
    linemarker.id = 2;
    linemarker.type = visualization_msgs::Marker::LINE_LIST;
    linemarker.scale.x = 0.02;
    linemarker.color.g = 1.0;
    linemarker.color.a = 1.0;

    for(size_t i = 0; i < cloud.size(); i ++)
    {
	geometry_msgs::Point p_v;
	p_v.x = cloud[i].x;
	p_v.y = cloud[i].y;
	p_v.z = 0;
//ROS_INFO("i = %d, x = %f, y = %f", i, p_v.x, p_v.y);
	linemarker.points.push_back(p_v);
    }

    publisher.publish(linemarker);
}

template <typename PointT>
void publishLineVisualMsg(ros::Publisher& publisher,
				const std::vector<PointT>& cloud,
				float r,
				float g,
				float b,
				std::string frameID)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cone";
    marker.id = 3;

    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for(size_t i = 0; i < cloud.size(); i ++)
    {
	geometry_msgs::Point p_v;
	p_v.x = cloud[i].x;
	p_v.y = cloud[i].y;
	p_v.z = 0;

	marker.points.push_back(p_v);
    }

    publisher.publish(marker);
}

//转换到绝对坐标系
void coneTrans(const pcl::PointXYZI& pi, pcl::PointXYZI& po, Twist test)
{
    PointType p;
    p.x = pi.x * test.rot_z.cos() - pi.y * test.rot_z.sin() + test.pos.x();
    p.y = pi.x * test.rot_z.sin() + pi.y * test.rot_z.cos() + test.pos.y();
    po.x = p.x;
    po.y = p.y;
//     po.x = pi.x;
//     po.y = pi.y;
//     po.z = pi.z;
//     po.intensity = pi.intensity;
// 
// //     rotateZXY(po, poseCurrent.rot_z, poseCurrent.rot_x, poseCurrent.rot_y);
//     rotateZXY(po, poseCurrent.rot_z, poseCurrent.rot_x, poseCurrent.rot_y);
// 
//     po.x += poseCurrent.pos.x();
//     po.y += poseCurrent.pos.y();
//     po.z += poseCurrent.pos.z();
}

//转换到相对坐标系
void coneAntiTrans(const pcl::PointXYZI& pi, pcl::PointXYZI& po, Twist& test)
{
    PointType p;
    p.x = pi.x * test.rot_z.cos() + pi.y * test.rot_z.sin() - (test.pos.x() * test.rot_z.cos() + test.pos.y() * test.rot_z.sin());
    p.y = -pi.x * test.rot_z.sin() + pi.y * test.rot_z.cos() - (-test.pos.x() * test.rot_z.sin() + test.pos.y() * test.rot_z.cos());   
//     po.y = ((pi.y - test.pos.y()) * test.rot_z.cos() - (pi.x - test.pos.x()) * test.rot_z.sin()) / (pow(test.rot_z.cos(), 2) - pow(test.rot_z.sin(), 2));
    po.x = p.x;
    po.y = p.y;
    
//     po.x = pi.x;
//     po.y = pi.y;
//     po.z = pi.z;
//     po.intensity = pi.intensity;
//     
//     po.x -= poseCurrent.pos.x();
//     po.y -= poseCurrent.pos.y();
//     po.z -= poseCurrent.pos.z();
// 
// //     rotateZXY(po, -poseCurrent.rot_z, -poseCurrent.rot_x, -poseCurrent.rot_y);
//     rotateZXY(po, -poseCurrent.rot_z, poseCurrent.rot_x, poseCurrent.rot_y);
}

void CANInit()
{
    /*
    const char  *szDevNode = DEFAULT_NODE;//define const pointer point to device name
    pcan_handle = LINUX_CAN_Open(szDevNode, O_RDWR | O_NONBLOCK);//use mapping function  
    */
}

void CANWrite(std::vector<PointType>& points_ways)
{
/*
    TPCANMsg msgOut;
    DWORD err;
    for(size_t i = 0; i < points_ways.size(); i++)
    {
	// Generate the outbound message
	msgOut.ID = 0x350 + i;
	msgOut.MSGTYPE = MSGTYPE_STANDARD;
	msgOut.LEN = 8;
	if(points_ways.size() > 0)
	{
	    msgOut.DATA[0] = 1;
	    msgOut.DATA[1] = (int)abs((-1*(points_ways[i].x)+2.02)*1000) >> 8 & 0xff;
	    msgOut.DATA[2] = (int)abs((-1*(points_ways[i].x)+2.02)*1000) & 0xff; // Set Status
	    msgOut.DATA[3] = (int)abs((points_ways[i].y)*1000) >> 8 & 0xff; // x high bit
	    msgOut.DATA[4] = (int)abs((points_ways[i].y)*1000) & 0xff; //
	    msgOut.DATA[5] = points_ways[i].x >=0?0:1;
	    msgOut.DATA[6] = points_ways[i].y >=0?1:0; // Status = Ready
	    msgOut.DATA[7] = 0;
	}
	else
	{
	    msgOut.DATA[0] = 0;
	    msgOut.DATA[1] = 0; // x high bit
	    msgOut.DATA[2] = 0; //
	    msgOut.DATA[3] = 0;
	    msgOut.DATA[4] = 0; // Set Status
	    msgOut.DATA[5] = 0;
	    msgOut.DATA[6] = 0; // Status = Ready
	    msgOut.DATA[7] = 0;
	}
	ROS_INFO("TargetoutToCan--> %d,%2x,%2x", i, msgOut.DATA[3], msgOut.DATA[4]);

	// Send the message
	//err = CAN_Write( &msgOut );
	err=CAN_Write(pcan_handle,&msgOut);
    }
*/  
}

void shutdown()
{
/*
    TPCANMsg msgOut;
    DWORD err;
    for(size_t i = 0; i < points_ways.size(); i++)
    {
	// Generate the outbound message
	msgOut.ID = 0x349;
	msgOut.MSGTYPE = MSGTYPE_STANDARD;
	msgOut.LEN = 8;
	
	    msgOut.DATA[0] = 1;
	    msgOut.DATA[1] = 2;
	    msgOut.DATA[2] = 3;
	    msgOut.DATA[3] = 4;
	    msgOut.DATA[4] = 5;
	    msgOut.DATA[5] = 6;
	    msgOut.DATA[6] = 7;
	    msgOut.DATA[7] = 8;

	// Send the message
	//err = CAN_Write( &msgOut );
	err=CAN_Write(pcan_handle,&msgOut);
    }
*/  
}
