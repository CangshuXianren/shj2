
#include <thread>
#include <fstream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFull(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFull1(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFull2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFull3(new pcl::PointCloud<pcl::PointXYZI>());
// pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
double filter_size_map_min;

class pcdsss
{
public:
	pcdsss(){}
	~pcdsss()
	{
		if (laserCloudFull1->points.size() > 0)
		{
		    string file_name = string("scans_all2.pcd");
			string all_points_dir(string("/home/dog/Documents/") + file_name);
			// string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
		    pcl::PCDWriter pcd_writer;
			*laserCloudFull += *laserCloudFull1;
			*laserCloudFull += *laserCloudFull2;
			// *laserCloudFull += *laserCloudFull3;
			cout << "laserCloudFull_all2->size(): " << laserCloudFull->size() << endl;
		    pcd_writer.writeBinary(all_points_dir, *laserCloudFull);

			laserCloudFull->clear();
		    file_name = string("scans_all3.pcd");
			all_points_dir = (string("/home/dog/Documents/") + file_name);
			// string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
		    pcl::PCDWriter pcd_writer2;
			*laserCloudFull += *laserCloudFull1;
			*laserCloudFull += *laserCloudFull3;
			cout << "laserCloudFull_all3->size(): " << laserCloudFull->size() << endl;
		    pcd_writer2.writeBinary(all_points_dir, *laserCloudFull);
		}
		else
			cout << "no enough points in all" << endl;

		if (laserCloudFull1->points.size() > 0)
		{
		    string file_name = string("scans_1.pcd");
			string all_points_dir(string("/home/dog/Documents/") + file_name);
			// string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
		    pcl::PCDWriter pcd_writer;
			cout << "laserCloudFull1->size(): " << laserCloudFull1->size() << endl;
		    pcd_writer.writeBinary(all_points_dir, *laserCloudFull1);
		}
		else
			cout << "no enough points in 1" << endl;

		if (laserCloudFull2->points.size() > 0)
		{
		    string file_name = string("scans_2.pcd");
			string all_points_dir(string("/home/dog/Documents/") + file_name);
			// string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
		    pcl::PCDWriter pcd_writer;
			cout << "laserCloudFull2->size(): " << laserCloudFull2->size() << endl;
		    pcd_writer.writeBinary(all_points_dir, *laserCloudFull2);
		}
		else
			cout << "no enough points in 2" << endl;

		if (laserCloudFull3->points.size() > 0)
		{
		    string file_name = string("scans_3.pcd");
			string all_points_dir(string("/home/dog/Documents/") + file_name);
			// string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
		    pcl::PCDWriter pcd_writer;
			cout << "laserCloudFull3->size(): " << laserCloudFull3->size() << endl;
		    pcd_writer.writeBinary(all_points_dir, *laserCloudFull3);
		}
		else
			cout << "no enough points in 3" << endl;
	}
};

void livox_pcl_cbk1(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *laserCloudFull1);
}

void livox_pcl_cbk2(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *laserCloudFull2);
}

void livox_pcl_cbk3(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *laserCloudFull3);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_save");
    ros::NodeHandle nh;

	nh.param<double>("leaf_size_map", filter_size_map_min, 0.1);
	// filter_size_map_min = 0.2;

	pcdsss ps;

    ros::Subscriber subLaserCloudFull1 = nh.subscribe<sensor_msgs::PointCloud2>
            ("/cloud_transformed1", 100000, livox_pcl_cbk1);
    ros::Subscriber subLaserCloudFull2 = nh.subscribe<sensor_msgs::PointCloud2>
            ("/cloud_transformed2", 100000, livox_pcl_cbk2);
    ros::Subscriber subLaserCloudFull3 = nh.subscribe<sensor_msgs::PointCloud2>
            ("/cloud_transformed3", 100000, livox_pcl_cbk3);

	ros::spin();
}
