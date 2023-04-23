#ifndef CS15_H
#define CS15_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <routing/Astar_searcher.h>
#include <custom_messages/mapmodel.h>
#include "geometry_msgs/PoseArray.h"
#include <custom_messages/planning_info.h>

#include "custom_messages/status15.h"
#include "custom_messages/chief_cmd.h"

#include "std_msgs/Int32.h"

/*差分计算函数
输入：一列数
输出：输出数列为输入数列两两之间的差值
*/
std::vector<double> vec_diff(const std::vector<double> &input);

/*累加计算函数
输入：一列数
输出：输出数列为输入数列从开始到对应位置的累加
*/
std::vector<double> cum_sum(const std::vector<double> &input);

class Spline
{
public:
    std::vector<double> x,y;               //x为弧长，y为对应的坐标
    std::vector<double> h;				  //y的步长
    std::vector<double> a,b,c,d;      //三次多项式的四个系数，注意a是最低次项，依次升高
    int nx;                     //区间数，即一共几段要拟合的曲线

    //默认构造函数
    Spline()
    {
    }
    /*普通构造函数，最终得到每个区间拟合的系数
    d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    */
    Spline(std::vector<double> x_,std::vector<double> y_):x(x_),y(y_),nx(x_.size()),h(vec_diff(x_)),a(y_)
    {
        //构造以系数c为未知量的矩阵方程并求解，公式易知，直接可输入此矩阵方程的系数矩阵A和解向量B
        Eigen::MatrixXd A;
        Eigen::VectorXd B,c_temp;
        A=calc_A();
        B=calc_B();
        c_temp=A.colPivHouseholderQr().solve(B);
        // 存储顺序默认使用列存储，可以使用data()函数获得存储数据的首地址
        double *c_pointer = c_temp.data();
        /* Map类用于通过C++中普通的连续指针或者数组 （raw C/C++arrays）来构造Eigen里的Matrix类，
            这就好比Eigen里的Matrix类的数据和raw C++array共享了一片地址，也就是引用。
            比如有个API只接受普通的C++数组，但又要对普通数组进行线性代数操作，那么用它构造为Map类，
            直接操作Map就等于操作了原始普通数组，省时省力。
            STL中不同容器之间是不能直接赋值的，assign（）可以实现不同容器但相容的类型赋值*/
        c.assign(c_pointer, c_pointer + c_temp.rows());
        //套公式求出系数b和d
        for(int i=0; i<nx-1; i++)
        {
            d.push_back((c[i+1] - c[i]) / (3.0 * h[i]));
            b.push_back((a[i+1] - a[i]) / h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
        }
    }

    //求t位置的0阶导数
    double calc(double t);
    //求t位置的1阶导数
    double calc_d(double t);
    //求t位置的2阶导数
    double calc_dd(double t);

private:
    Eigen::MatrixXd calc_A();

    Eigen::VectorXd calc_B();

    /*递归二分法找根*/
    int bisect(double t, int start, int end);
};

class Spline2D
{
public:
    Spline sx;
    Spline sy;
    std::vector<double> s;                  //弧长
    //构造函数
    Spline2D(){};
    Spline2D(std::vector<double> x,std::vector<double> y)
    {
        s=calc_s(x,y);
        sx=Spline(s,x);
        sy=Spline(s,y);
    }

    //位置计算函数
    std::array<double,2> calc_postion(double s_t);
    //曲率计算函数
    double calc_curvature(double s_t);
    //航向角计算函数
    double calc_yaw(double s_t);

    //弧长计算函数，输出数列为从起点到当前位置的弧长
    std::vector<double> calc_s(std::vector<double> x, std::vector<double> y);
};

struct SplineCarState
{
    double x;
    double y;
    double yaw;
    double speed;
    double angular_speed;
    // SplineCarState(){};
    // SplineCarState(double x_, double y_, double yaw_, double speed_, double angular_speed_):
    //     x(x_), y(y_), yaw(yaw_), speed(speed_), angular_speed(angular_speed_)
    // {}
    SplineCarState & operator =(const SplineCarState &input); 
};

class CSRefPath
{
public:
    CSRefPath(){};
    ~CSRefPath(){};
    void CSclear();
    void load(Spline2D &spline, double overall_length);
    void joint(CSRefPath &path);
    void copypath(CSRefPath &path, int index);
    
    CSRefPath& operator=(CSRefPath& input);

    std::vector<double> r_x;
    std::vector<double> r_y;
    std::vector<double> r_yaw;
    std::vector<double> r_curvature;
    std::vector<double> r_s;
};

// plannercore
class CScore15
{
public:
    CScore15(int recv_ID, int recv_target_ID) {
        ID = recv_ID;
        target_ID = recv_target_ID;
        keeper.resize(2);
    }
    void FrontCallback(const nav_msgs::Odometry::ConstPtr& front_msg);
    void LeaderCallback(const nav_msgs::Odometry::ConstPtr& leader_msg);
    void SelfCallback(const nav_msgs::Odometry::ConstPtr& self_msg);
    void ObsCallback(const custom_messages::mapmodel::ConstPtr& obs_msg);
    bool CollisionCheck(const CSRefPath &r_path, const SplineCarState &carstate, const AstarPathFinder* tmpmap, int dynamic_button, int static_button); // mode=0,static map; mode=1, static map and dynamic obstacle
    void planning();

    void ACallback(const custom_messages::status15::ConstPtr& p_msg);
    void BCallback(const custom_messages::status15::ConstPtr& p_msg);
    void CCallback(const custom_messages::status15::ConstPtr& p_msg);
    void ReconfigurationCallback(const std_msgs::Int32::ConstPtr& reconfiguration_msg);
    // void ChiefCallback(const custom_messages::chief_cmd::ConstPtr& p_msg);

    bool GetVehicleInfo(int mode, int index, geometry_msgs::PoseStamped& out); // mode=0:front, mode=1:leader
    bool GetVehiclemyInfo(int index, SplineCarState& out);
    void Status15toRosodom(const custom_messages::status15::ConstPtr& input, SplineCarState& output);

    int reconfiguration_flag = 0; // 0-false, 1-add(particular unit), 2-quit
    bool add_reconfiguration_flag = false;

    void UpdateKeeper();
    int findMatchpoint(double x, double y, CSRefPath& input_trajectory);
    bool add_check();
    bool add_lateral_check();
    bool add_yaw_check();

    SplineCarState fmodom[3];

    std::vector<SplineCarState> keeper;

    SplineCarState mystate;
    SplineCarState front;
    int ID;
    int target_ID;
    int request_numnber2_;

    custom_messages::mapmodel obstacle;
    bool self_initial = false;
    bool front_initial = false;
    // bool leader_initial = false;
};



#endif