#ifndef _MPC_ZCY_
#define _MPC_ZCY_

#include<map>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include<nav_msgs/Path.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include <queue>
#include <time.h>
#include "quadprog/QuadProg++.h"
//#include "quadprog/Array.h"
#include "custom_messages/vehicle_status.h"
#include "math.h"
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iterator>

#define Unit 1.0
#define Zero 0.0

namespace quadprogpp{

class pathpoint{
public:
    double x_r, y_r, fai_r, vx_r, vw_r;
};

class mpc{

public:

    mpc();
    ~mpc();

    //用于初始化一大堆矩阵
    bool init(int _Nx, int _Nu, int _Np, int _Nc, int _Row, double delta_v, double delta_w, int q1,  int q2,  int q3);

    //订阅目标并接收状态量
    //写了一个重载
    //第一个是接收一个简单的目标
    //第二个是接收nav_msgs里的路径信息
    //同时包含序列号，时间，位置和姿态（四元数），好处是rviz里边这个能显示为一条连续的路径
   // void messageCallback(const tf_follower::Target::ConstPtr &msg);

   void PointCallBack(const nav_msgs::Path &msg);
   void velocityCallback(const  nav_msgs::Odometry &msg);
   void messageCallback(const nav_msgs::Odometry &msg);

    //发布控制量
    void publishVel(double angleCommand,double speedCommand);
    void publishVel(double angleCommand,double speedCommand,double a);


    //控制结果输出器
    //需要向这里传入一个状态量u，包含x,y,th三个参数
    //这里返回的是自定的消息类型，内容为控制量
    //double mpc_output(double &x_r,double &y_r, double &th_r,double &x,double &y, double &th );
    custom_messages::vehicle_status mpc_output(float &T, custom_messages::vehicle_status &Acon, custom_messages::vehicle_status &con, custom_messages::vehicle_status &ref, custom_messages::vehicle_status &u, double vmin, double vmax, double wmin, double wmax);


private:
    double L;                                                 //轴距，用于阿克曼转向的计算,x-tark小车约0.18m
    int Nx;                                                       //状态量个数
    int Nu;                                                       //控制量个数
    int Np;                                                      //预测时域
    int Nc;                                                       //控制时域
    int Row;                                                   //松弛因子

    Matrix<double>U;                              //用于储存控制量，列数为控制量个数，Nu*1
    Matrix<double>kesi;                          //由状态量和控制量共同组成的向量，目的是使二次规划问题中约束量转化为控制量增量(Nx+Nu)*1
    Matrix<double>q;                               //用于设置状态量对应的权重矩阵，所以是一个Nx*Nx的对角矩阵
    Matrix<double>Q;                              //权重矩阵，由q在预测时域上扩展生成，也是方阵，行列数为Nx*Np
    Matrix<double>R;                               //用于控制量偏差的权重矩阵，目前是0

    //以下是控制量是前轮转角的参数
    // Matrix<double>a;                               //线性化模型后线性项的系数
    // Matrix<double>b;                               //同上
    Matrix<double>bb;                                 //用于填充分块矩阵的单位矩阵
    // Matrix<double>A;                               //构造的增量二次规划目标函数对应的线性项系数
    // Matrix<double>B;                               //同上
    // Matrix<double>C;                              //同上
    // Matrix<double>PHI;                         //递推公式kesi项的系数
    // Matrix<double>THETA;                   //递推公式控制增量项的系数
    //以下是控制量是角速度的参数
    Matrix<double>a_w;                               //线性化模型后线性项的系数，
    Matrix<double>b_w;                               //同上
    Matrix<double>A_w;                               //由a_w生成，在预测和控制时域内扩展而成
    Matrix<double>B_w;                               //同上
    Matrix<double>C_w;                              //由状态变量生成输出变量的转化矩阵

    Matrix<double>PHI_w;                         //递推公式kesi项的系数
    Matrix<double>THETA_w;                   //递推公式"控制误差增量项"的系数
    Matrix<double>CA1;                               //生成PHI_w的中间变量
    Matrix<double>CAB;                               //生成THETA_w的中间变量

    Matrix<double>H;                                   //二次规划求解器输入的系数
    Matrix<double>H1;                                //生成H使用的中间变量

    Matrix<double>Mf;                                 //与f一样，只是，是个矩阵
    Matrix<double>Mf1;                              //生成Mf的中间变量
    Matrix<double>error;                           //递推公式的第一项
    Vector<double>f;                                    //二次规划求解器输入的系数，注意是一个向量，
                                                                            //维度为控制量在控制时域上的扩展再加对应着松弛因子的1个0

    //以下是比较头疼的一部分，涉及约束变量相关矩阵的 
    //最终获得的是CI和ci0，约束形式如下：
    // 
    Matrix<double>A_t;                                //首先生成一个和控制时域大小一致的单位下三角矩阵
    Matrix<double>A_I1;                              //初始化成了一个Nu*Nu的单位矩阵
    Matrix<double>A_I;                               //以上两者做克罗内克积，即把A_I1在控制时域上进行扩展
    Matrix<double>Acons_Zero;             //Np*1的矩阵
    Matrix<double>Acons;                         //A_I和Acons_Zero拼成
    Matrix<double>CI0;                               //由Acons转置并取负构成，用于限制次增量带来的总量增加别超了

    Matrix<double>CI1_1;                          //填入约束量
    Matrix<double>CI1;                              //把约束量在控制时域上扩展，用于限制增量在一个范围内

    Matrix<double>CI;                                  //把CI0和CI1填入

    Matrix<double>Ut_unit;                      //Nc行一列，用于控制量的扩展
    Matrix<double>Ut;                                 //这是一个基准，是对控制量误差在控制时域上的扩展

    Matrix<double>umin;                          //和U类似也是个Nu*1的矩阵
    Matrix<double>umax;
    Matrix<double>Umin;                         //和Ut类似，是umin在控制时域上的扩展
    Matrix<double>Umax;

    Matrix<double>ci0_1;                           //用于设置增量本身的范围限制
    Matrix<double>ci0_0;                          //设置分块矩阵，用于限制总量的范围
    Matrix<double>ci0_m;                        //同ci0，区别就是这个是矩阵
    Vector<double>ci0;                               //含有范围限制的输入向量

    Vector<double>X;                                  //储存二次规划输出的结果 ，按照控制量个数顺序储存

    public:

    // ros::Publisher pubMessage;
  
    const double maxSpeed=0.6;
    const double maxASpeed=0.9;

    custom_messages::vehicle_status control_aim;
    int pointNum;
    //float k=0.1;
    //float preview_dis=0;
    float t;
    float T;
    bool if_init;
    double Roll,Pitch,Yaw;
    tf::Quaternion quat;
    nav_msgs::Path runpath;

};

}


#endif
