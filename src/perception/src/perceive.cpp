#include "perceive.h"
using namespace std;

bool offset_1 = false; // manually offset
bool offset_2; // double offset from mapmerge
double offset_x, offset_y, offset_yaw;

nav_msgs::OccupancyGrid mymapping;

ros::Publisher obstacle_pub, obs_vis_pub;

custom_messages::vehicle_status now_pos;

// 转换到绝对坐标系
inline void coneTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d test)
{
    po[0] = pi[0] * cos(test[2]) - pi[1] * sin(test[2]) + test[0];
    po[1] = pi[0] * sin(test[2]) + pi[1] * cos(test[2]) + test[1];
}

//转换到相对坐标系
inline void coneAntiTrans(const Eigen::Vector2d& pi, Eigen::Vector2d& po, Eigen::Vector3d& test)
{
    po[0] = pi[0] * cos(test[2]) + pi[1] * sin(test[2]) - (test[0] * cos(test[2]) + test[1] * sin(test[2]));
    po[1] = -pi[0] * sin(test[2]) + pi[1] * cos(test[2]) - (-test[0] * sin(test[2]) + test[1] * cos(test[2]));   
}

void visObstacle(vector<Eigen::Vector3d> nodes){
    visualization_msgs::MarkerArray vis_array;
    for(size_t i = 0; i < nodes.size(); i++)
    {
        visualization_msgs::Marker node_vis;
        node_vis.header.frame_id = "map";
        node_vis.header.stamp = ros::Time::now();

        node_vis.ns = "perceive/obs";
        
        // boat
        node_vis.type = visualization_msgs::Marker::CYLINDER;
        // node_vis.type = visualization_msgs::Marker::CUBE;
        node_vis.action = visualization_msgs::Marker::ADD;
        node_vis.id = i; // 分配给marker的唯一的id

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;

        // 调整颜色
        node_vis.color.a = 0.8;
        node_vis.color.r = 0.9;
        node_vis.color.g = 0.1;
        node_vis.color.b = 0.1;
        node_vis.scale.x = 2*nodes[i](2);
        node_vis.scale.y = 2*nodes[i](2);
        //boat
        node_vis.scale.z = 1.0;
        // node_vis.scale.z = 3.0;
        node_vis.pose.position.x = nodes[i](0);
        node_vis.pose.position.y = nodes[i](1);

        vis_array.markers.push_back(node_vis);
    }
    obs_vis_pub.publish(vis_array);
}

void SlamCallback(const nav_msgs::OccupancyGrid::ConstPtr& slam_msg){
    if(slam_msg->data.empty()) return;
    mymapping = *slam_msg;
}

// void LocationCallback(const custom_messages::vehicle_status::ConstPtr& self_loc_msg){
//     now_pos.xPos = self_loc_msg->xPos;
//     now_pos.yPos = self_loc_msg->yPos;
//     now_pos.yaw = self_loc_msg->yaw;
// }
// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     if(msg == nullptr) return;
//     now_pos.xPos = msg->pose.pose.position.x;
//     now_pos.yPos = msg->pose.pose.position.y;
//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
//     double Roll, Pitch, Yaw;
//     tf::Matrix3x3(quat).getRPY(Roll, Pitch, Yaw);
//     now_pos.yaw = Yaw;
// }
void iniodomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(msg == nullptr) return;
    now_pos.xPos = msg->pose.pose.position.x;
    now_pos.yPos = msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double Roll, Pitch, Yaw;
    tf::Matrix3x3(quat).getRPY(Roll, Pitch, Yaw);
    now_pos.yaw = Yaw;
}

void PerceiveCallback(const sensor_msgs::PointCloud2::ConstPtr& perceive_msg){
    pcl::PointCloud<PointType>::Ptr laserCloudIn (new pcl::PointCloud<PointType>); // 对x字段的滤波
    pcl::PointCloud<PointType>::Ptr laserCloudIn2 (new pcl::PointCloud<PointType>); // 对y字段的滤波
    pcl::PointCloud<PointType>::Ptr laserCloudIn3 (new pcl::PointCloud<PointType>); // 对z字段的滤波
    pcl::PointCloud<PointType>::Ptr filterCloudLatest (new pcl::PointCloud<PointType>); // 三层过滤之后最终的点云
    pcl::PointCloud<PointType>::Ptr groundCloudLatest (new pcl::PointCloud<PointType>); // 地面点云
    pcl::PointCloud<PointType>::Ptr objectCloudLatest (new pcl::PointCloud<PointType>); // 目标物点云
    pcl::PointCloud<PointType>::Ptr coneCouldLatest (new pcl::PointCloud<PointType>); // 将目标物模型化后的障碍物点云
    
    pcl::fromROSMsg(*perceive_msg, *laserCloudIn);

    // fliter
    pcl::PassThrough<PointType> pass1;
    pcl::PassThrough<PointType> pass2;
    pcl::PassThrough<PointType> pass3;
    pass1.setInputCloud (laserCloudIn);
    pass1.setFilterFieldName ("x");
    pass1.setFilterLimits (-13.0, 13.0);
    pass1.filter(*laserCloudIn2);

    pass2.setInputCloud (laserCloudIn2);
    pass2.setFilterFieldName ("y");
    pass2.setFilterLimits (-10.0, 10.0);
    pass2.filter (*laserCloudIn3);

    pass3.setInputCloud (laserCloudIn3);
    pass3.setFilterFieldName ("z");
    pass3.setFilterLimits (-1.5, 0.5);
    pass3.filter (*filterCloudLatest);

    // Plane Model Segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (filterCloudLatest);
    seg.segment (*inliers, *coefficients);
	
	// float A, B, C, D;
	// A = coefficients->values[0];
	// B = coefficients->values[1];
	// C = coefficients->values[2];
	// D = coefficients->values[3];
// 	ROS_INFO("Plane value : a = %f b = %f c = %f d = %f", 
// 			 coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    // 提取地面
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (filterCloudLatest);
    extract.setIndices (inliers);
    extract.filter (*groundCloudLatest);
    // 提取除地面外的物体
    extract.setNegative (true);
    extract.filter (*objectCloudLatest);
    
    // ROS_INFO("cluster ...");
    std::vector<pcl::PointIndices> cluster_indices;
    // cluster
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    pcl::EuclideanClusterExtraction<PointType> ec;
    cluster_indices.clear();
    tree->setInputCloud(objectCloudLatest);
    ec.setClusterTolerance (0.2);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (objectCloudLatest);
    ec.extract (cluster_indices);
	// std::cout << "cluster size: " << cluster_indices.size() << std::endl;

    //push cone points
    std::vector<PointType> obj;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        PointType min_point_OBB;
        PointType max_point_OBB;
        PointType position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;

        pcl::PointIndices::Ptr inlier (new pcl::PointIndices);
        *inlier = *it;
        pcl::MomentOfInertiaEstimation <PointType> feature_extractor;
        feature_extractor.setInputCloud(objectCloudLatest);
        feature_extractor.setIndices(inlier);
        feature_extractor.compute();
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        float dx = max_point_OBB.x - min_point_OBB.x;
        float dy = max_point_OBB.y - min_point_OBB.y;
        float dz = max_point_OBB.z - min_point_OBB.z;
        if ((dx > 1.5) || (dy > 1.5)) continue;
    
        PointType p;
        p.x = position_OBB.x;
        p.y = position_OBB.y;
        p.z = sqrt(pow(p.x, 2) + pow(p.y, 2)); // 点云质心到自身的距离
        p.intensity = (dx > dy ? dx : dy) * 0.5; // 最大半径，储存点云质心到最远点的半径

		// int num = 0;
		// for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		// {
		// 	p.x += objectCloudLatest->points[*pit].x;
		// 	p.y += objectCloudLatest->points[*pit].y;
		// 	num++;
		// }
		// // 计算点云质心
        // p.x = p.x / float(num);
        // p.y = p.y / float(num);

		// float range = 0.0;
		// for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        // {
        //     range = pow(objectCloudLatest->points[*pit].x - p.x, 2) + pow(objectCloudLatest->points[*pit].y - p.y, 2);
        //     if (range > p.intensity) p.intensity = range;
        // }
        // if (p.intensity == 0.0)
		// {
		// 	// ROS_WARN("no radius");
		// 	continue;
		// }
		// p.intensity = sqrt(p.intensity);
		// p.z = sqrt(p.x*p.x + p.y*p.y);
        coneCouldLatest->points.push_back(p);
        obj.push_back(p);

        // std::cout << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << std::endl;
        // std::cout << it->indices.size() << std::endl;
    }
    // if (cluster_indices.size() != coneCouldLatest->points.size())
    //     ROS_WARN("cluster_indices.size != coneCouldLatest->points.size");
    std::cout << "[perception] : obj.size ; " << obj.size() <<std::endl;
    custom_messages::mapmodel obstacle_msg;
    vector<Eigen::Vector3d> obs_vis;
    geometry_msgs::Point tmp;
    Eigen::Vector3d tmpvis;
    for(auto &my : obj){
        Eigen::Vector2d obs_in_vehicle, obs_in_myodom, obs_in_map;
        obs_in_vehicle << my.x, my.y;
        Eigen::Vector3d move1, move2;
        move1 << now_pos.xPos, now_pos.yPos, now_pos.yaw;
        move2 << offset_x, offset_y, offset_yaw;
        coneTrans(obs_in_vehicle, obs_in_myodom, move1);
        coneTrans(obs_in_myodom, obs_in_map, move2);
        tmp.x = obs_in_map(0);
        tmp.y = obs_in_map(1);
        tmp.z = my.intensity;
        obstacle_msg.mapinfo.push_back(tmp);

        tmpvis(0) = obs_in_map(0);
        tmpvis(1) = obs_in_map(1);
        tmpvis(2) = my.intensity;
        obs_vis.push_back(tmpvis);
    }
    obstacle_pub.publish(obstacle_msg);
    visObstacle(obs_vis);
}

void TestCallback(const custom_messages::mapmodel::ConstPtr& perceive_msg){
    vector<Eigen::Vector3d> obs_vis;
    Eigen::Vector3d tmpvis;
    custom_messages::mapmodel obstacle_msg;
    geometry_msgs::Point tmp;
    for(auto &my : perceive_msg->mapinfo){
        tmpvis(0) = my.x;
        tmpvis(1) = my.y;
        tmpvis(2) = my.z;
        obs_vis.push_back(tmpvis);

        tmp.x = my.x;
        tmp.y = my.y;
        tmp.z = my.z;
        obstacle_msg.mapinfo.push_back(tmp);
    }
    visObstacle(obs_vis);
    obstacle_pub.publish(obstacle_msg);
}

void OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& offset_msg){
    if(offset_1 && offset_2) return;
    offset_x = offset_msg->pose.pose.position.x;
    offset_y = offset_msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(offset_msg->pose.pose.orientation, quat);
    double roll, pitch;
    tf::Matrix3x3(quat).getRPY(roll, pitch, offset_yaw);
    if (offset_1 == false) {
        offset_1 = true;
    } else if (offset_1 == true) {
        offset_2 = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "perceive_node");
    ros::NodeHandle nh;

    nh.param<bool>("perceive/offset_2", offset_2, true);

    // ros::Subscriber slam_ret_sub = nh.subscribe("slam_ret", 1, SlamCallback);
    ros::Subscriber offset_sub = nh.subscribe("initialpose", 2, OffsetCallback);
    ros::Subscriber mypoint = nh.subscribe("self_location", 2, iniodomCallback);
    ros::Subscriber lidar_sub = nh.subscribe("velodyne_points", 1, PerceiveCallback);
    ros::Subscriber test_sub = nh.subscribe("virtual_obstacle_info", 1, TestCallback);
    obstacle_pub = nh.advertise<custom_messages::mapmodel>("obstacle_info", 1);
    obs_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("dynamic_obs_vis", 1);
    ros::spin();

    return 0;
}