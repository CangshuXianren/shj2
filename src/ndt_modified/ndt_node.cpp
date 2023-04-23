
#include <ros/ros.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <voxel_grid_large.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef pcl::PointXYZI PointType;

pcl::PointCloud<PointType>::Ptr map0(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr map1(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr map2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr map3(new pcl::PointCloud<PointType>());
double filter_size_map_min;
int merge_num;

std_msgs::Header cloudHeader1, cloudHeader2, cloudHeader3;

bool hasCloud1 = false, hasCloud2 = false, hasCloud3 = false, hasTrans = false, hasTrans3 = false;
// Eigen::Matrix4f init_guess_last (Eigen::Matrix4f::Identity());
// Eigen::Matrix4f init_guess_last3 (Eigen::Matrix4f::Identity());

uint8_t status = 0;
Eigen::Vector3f odom;
geometry_msgs::PoseWithCovarianceStamped offset0;

ros::Publisher pubOffset1, pubOffset2, pubOffset3;

class pcdsave
{
public:
	pcdsave(){}
	~pcdsave()
	{
		ROS_INFO("pcd saving ...");
		pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
		// *laserCloudIn += *laserCloudFull1;
		// *laserCloudIn += *laserCloudFull2;
		// *laserCloudIn += *laserCloudFull3;
		pcl::io::savePCDFileBinary("/home/yxt/Documents/map.pcd", *laserCloudIn);
		ROS_INFO("pcd saved: %d", laserCloudIn->points.size());
	}
};

void laserCloud1Handler0(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if (status == 2) pcl::fromROSMsg(*msg, *map0);
}

void laserCloud1Handler1(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if (status == 2) pcl::fromROSMsg(*msg, *map1);
}

void laserCloud1Handler2(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if (status == 2) pcl::fromROSMsg(*msg, *map2);
}

void laserCloud1Handler3(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if (status == 2) pcl::fromROSMsg(*msg, *map3);
}

void mergeMap(uint8_t i)
{
	Eigen::Matrix4f init_guess (Eigen::Matrix4f::Identity());
	init_guess.block(0,3,3,1) = Eigen::Vector3f(-0.6*i, 0., 0.);

	//初始化正态分布变换（NDT）
	pcl::NormalDistributionsTransform<PointType, PointType> ndt;
	//设置依赖尺度NDT参数
	//为终止条件设置最大转换差异
	ndt.setTransformationEpsilon (0.01);
	//为More-Thuente线搜索设置最大步长
	ndt.setStepSize (0.1); // 0.1 0.2
	//设置NDT网格结构的分辨率（VoxelGridCovariance）
	ndt.setResolution (1.0);
	//设置匹配迭代的最大次数
	ndt.setMaximumIterations (70); // 35 70
	// 设置要配准的点云
	if (i == 1)
		ndt.setInputSource (map1);
	else if (i == 2)
		ndt.setInputSource (map2);
	else if (i == 3)
		ndt.setInputSource (map3);
	else
		ROS_ERROR("merge error");
	//设置点云配准目标
	ndt.setInputTarget (map0);//第一次扫描的结果
	//计算需要的刚体变换以便将输入的点云匹配到目标点云
	pcl::PointCloud<PointType>::Ptr output_cloud (new pcl::PointCloud<PointType>);
	ndt.align (*output_cloud, init_guess);

	Eigen::Matrix4f trans = ndt.getFinalTransformation ();

	// Eigen::Isometry3f trans_ndt;
	// trans_ndt.rotate(trans.block(0,0,3,3));
	// trans_ndt.pretranslate(trans.block(0,3,3,1));
	// Eigen::Isometry3f offset_tr;
	// offset_tr.rotate(Eigen::Quaternionf(offset0.pose.pose.orientation.w(),
	// 					offset0.pose.pose.orientation.x(),
	// 					offset0.pose.pose.orientation.y(),
	// 					offset0.pose.pose.orientation.z()).toRotationMatrix());
	// offset_tr.pretranslate(Eigen::Vector3f(offset0.pose.pose.position.x(),
	// 					offset0.pose.pose.position.y(),
	// 					offset0.pose.pose.position.z()));

	Eigen::Quaternionf q(Eigen::Matrix3f(trans.block(0,0,3,3)));
	std::cout << i << std::endl;
	std::cout << "trans: " << trans(0,3) << ", " << trans(1,3) << ", " << trans(2,3) << std::endl;
	std::cout << "rot: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;

	Eigen::Quaternionf q0 (offset0.pose.pose.orientation.w,
							offset0.pose.pose.orientation.x,
							offset0.pose.pose.orientation.y,
							offset0.pose.pose.orientation.z);
	q = q * q0;

	geometry_msgs::PoseWithCovarianceStamped offset;
	offset.pose.pose.position.x = offset0.pose.pose.position.x + trans(0,3);
	offset.pose.pose.position.y = offset0.pose.pose.position.y + trans(1,3);
	offset.pose.pose.position.z = offset0.pose.pose.position.z + trans(2,3);
	offset.pose.pose.orientation.w = q.w();
	offset.pose.pose.orientation.x = q.x();
	offset.pose.pose.orientation.y = q.y();
	offset.pose.pose.orientation.z = q.z();

	switch (i)
	{
	case 1:
		pubOffset1.publish(offset);
		break;
	case 2:
		pubOffset2.publish(offset);
		break;
	case 3:
		pubOffset3.publish(offset);
		break;
	default:
		break;
	}
}

void odomHandler(const nav_msgs::OdometryConstPtr &msg)
{
	Eigen::Vector3f pos;
	pos << msg->pose.pose.position.x,
		   msg->pose.pose.position.y,
		   msg->pose.pose.position.z;
	
	if ((status == 0) && (pos.norm() > 2.5)) status = 1;
	if ((status == 1) && (pos.norm() < 2.)) status = 2;

	odom = pos;

	if (status == 2) {
		while (1) {
			if ((map0->size() > 1000) &&
				(map1->size() > 1000) &&
				(map2->size() > 1000) &&
				(map3->size() > 1000)) {
				mergeMap(1);
				mergeMap(2);
				mergeMap(3);

				ros::shutdown();
			}
		}
	}
}

void OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	offset0 = *msg;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ndt_node");
    ros::NodeHandle nh;

//	pcdsave pcd_save;
	
	double leafsize, step_size, step_size3;
	int max_ite;

	nh.param<double>("leaf_size_map", filter_size_map_min, 0.1);
	nh.param<double>("leaf_size_ndt", leafsize, 0.2);
	nh.param<double>("step_size", step_size, 0.2);
	nh.param<double>("step_size3", step_size3, 0.2);
	nh.param<int>("max_ite", max_ite, 30);
	nh.param<int>("merge_num", merge_num, 30);
	
	ros::Subscriber subLaserCloud0 = nh.subscribe<sensor_msgs::PointCloud2>
		("map0", 1, laserCloud1Handler0);
	ros::Subscriber subLaserCloud1 = nh.subscribe<sensor_msgs::PointCloud2>
		("map1", 1, laserCloud1Handler1);
	ros::Subscriber subLaserCloud2 = nh.subscribe<sensor_msgs::PointCloud2>
		("map2", 1, laserCloud1Handler2);
	ros::Subscriber subLaserCloud3 = nh.subscribe<sensor_msgs::PointCloud2>
		("map3", 1, laserCloud1Handler3);
	ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>
		("odom", 10, odomHandler);
	ros::Subscriber offset_sub = nh.subscribe("initialpose0", 2, OffsetCallback);
	
	pubOffset1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose1", 1);
	pubOffset2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose2", 1);
	pubOffset3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose3", 1);
	
	ros::spin();
	
	return 0;
}
