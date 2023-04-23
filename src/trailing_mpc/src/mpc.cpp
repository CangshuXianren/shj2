#include "trailing_mpc/mpc.h"

namespace quadprogpp{

mpc::mpc(){}
mpc::~mpc(){}

bool mpc::init(int _Nx, int _Nu, int _Np, int _Nc, int _Row, double delta_v, double delta_w, int q1,  int q2,  int q3){
    
    Nx = _Nx;
    Nu = _Nu;
    Np = _Np;
    Nc = _Nc;
    Row = _Row;

    //ROS_ERROR("%s","inout1.2");
    U.resize(Zero,Nu,1);
    //ROS_ERROR("%s","inout1.3");
    kesi.resize(Zero,Nx+Nu,1);
    //ROS_ERROR("%s","inout2");
    //以上是状态量和控制量相关的向量   
    q.resize(Zero,Nx,Nx);
    Q.resize(Zero,Nx*Np,Nx*Np);
    R.resize(Zero,Nu*Nc,Nu*Nc);
    // a.resize(MType::DIAG, Unit, Zero,Nx,Nx);
    // b.resize(Zero,Nx,Nu);
    a_w.resize(MType::DIAG, Unit, Zero,Nx,Nx);
    b_w.resize(Zero,Nx,Nu);
    //ROS_ERROR("%s","inout3");
    //以上对应的是《无人驾驶模型预测控制》书中的（4.6）的参数
    //以下省略书名
    // A.resize(MType::DIAG,Unit,Zero,Nx+Nu,Nx+Nu);
    // B.resize(Zero,Nx+Nu,Nu);
    // C.resize(MType::DIAG,Unit,Zero,Nx,Nx+Nu);
    //A_w.resize(MType::DIAG,Unit,Zero,Nx+Nu,Nx+Nu);
    A_w.resize(Zero,Nx+Nu,Nx+Nu);
    B_w.resize(Zero,Nx+Nu,Nu);
    bb.resize(MType::DIAG, Unit, Zero,Nu,Nu);
    C_w.resize(MType::DIAG,Unit,Zero,Nx,Nx+Nu);
    //以上对应的是书中（4.10）的参数
    PHI_w.resize(Zero,Np*Nx,Nx+Nu);
    THETA_w.resize(Zero,Np*Nx,Nc*Nu);
    //以上对应的是书中（4.12）的参数
    H.resize(Zero,Nu*Nc+1,Nu*Nc+1);
    Mf.resize(1,Nu*Nc+1);
    f.resize(Nu*Nc+1);
    //以上对应的是书中（4.19）的参数
    CI.resize(Zero, Nu*Nc+1, Nu*Nc*2+Nu*Nc*2);
	CI0.resize(Zero, Nu*Nc+1, Nu*Nc*2);
	Acons.resize(Zero, Nu*Nc*2, Nu*Nc+1);
	A_t.resize(Zero, Nc, Nc);
	A_I1.resize(MType::DIAG, Unit, Zero, Nu, Nu);
	Acons_Zero.resize(Zero, Nu*Nc, 1);
	CI1.resize(Zero, Nu*Nc+1, Nu*Nc*2);
	CI1_1.resize(Zero, Nu, Nu*2);
	ci0.resize(Zero, Nu*Nc*2+Nu*Nc*2);
	ci0_m.resize(Zero, Nu*Nc*2+Nu*Nc*2, 1);
	ci0_0.resize(Zero, Nu*Nc*2, 1);
	//ci0_1.resize(0.05, Nu*Nc*2, 1);
    ci0_1.resize(Zero,Nu*Nc*2,1);
    for(int i=0;i<Nu*Nc*2;i=i+4)
    {
        ci0_1[i][0] = delta_v;
        ci0_1[i+1][0] = delta_v;
        ci0_1[i+2][0] = delta_w;
        ci0_1[i+3][0] = delta_w;
    }

	Ut_unit.resize(Unit, Nc, 1);

	umin.resize(2, 1);
	umax.resize(2, 1);

	X.resize(Zero, Nu*Nc+1);

	q[0][0] = q1;
	q[1][1] = q2;
	q[2][2] = q3;
//	q << 10, 0 ,   0,
//		       0,  10,  0,
//		       0,  0 ,   1;

	for(int i=0; i<Nx*Np; i++){
		if(i%Nx == 0){
			Q[i][i] = q[0][0];
		}else if(i%Nx == 1){
			Q[i][i] = q[1][1];
		}else if(i%Nx== 2){
			Q[i][i] = q[2][2];
		}
	}

    //@jyf 这里把R权重设为了0，以后可能要改
	R *= 0;

	return true;
}

//参数说明：Acon储存的是参考控制量
//                      Bcon   储存的是上一时刻控制量
//                      r         储存的是参考的状态量
//                      u        储存的是当前的状态量
//tf_follower::SSpeed mpc::mpc_output(double &x_r, double &y_r, double &th_r, double &x,double &y, double &th){
custom_messages::vehicle_status mpc::mpc_output(float &T, custom_messages::vehicle_status &Acon, custom_messages::vehicle_status &Bcon, custom_messages::vehicle_status &r, custom_messages::vehicle_status &u, double vmin, double vmax, double wmin, double wmax)
{

    //获取参考控制量：车速和角速度
    float vd1=Acon.speed;              
    float vd2=Acon.angular_speed;              
    
    //建立状态控制向量kesi
    //当前时刻状态量和参考状态量的差值
    kesi[0][0]=u.xPos-r.xPos;
    kesi[1][0]=u.yPos-r.yPos;
    kesi[2][0]=u.yaw-r.yaw;
    //上一时刻控制量和参考控制量的差值
    kesi[3][0]=U[0][0];
    kesi[4][0]=U[1][0];

    // //如果控制量是驱动轮速度和前轮转角，具有如下形式
    // //a<<1,0,-vd1*sin(th_r)
    // a[0][2]=;
    // a[1][2]=;
    // //设置分跨矩阵如下
    // A.setblock();
    // A.setblock();

    //如果控制量是驱动轮速度和角速度，具有如下形式
    //a<<1,0,-vd1*sin(r.th)*T
    //        0,1,vd1*cos(r.th)*T
    //        0,0,1
    a_w[0][2]=-vd1*sin(r.yaw)*T;
    a_w[1][2]=vd1*cos(r.yaw)*T;
    //std::cout<<"a_w:"<<a_w<<std::endl;

    //b<<cos(r.th)*T  0
    //        sin(r.th)*T  0
    //        0                    T
    b_w[0][0]=cos(r.yaw)*T;
    b_w[1][0]=sin(r.yaw)*T;
    b_w[2][1]=T;
    //std::cout<<"b_w:"<<b_w<<std::endl;

    //设置分跨矩阵如下
    //最终表达式的线性项系数形式如下
    //A_w<<a(3*3)b(3*2)
    //              0          i(2*2)
    A_w.setblock(0,0,a_w);
    A_w.setblock(0,3,b_w);
    A_w.setblock(3,3,bb);
    //B_w=<<b(3*2)
    //                 i(2*2)
    B_w.setblock(0,0,b_w);
    B_w.setblock(3,0,bb);

    //最终递推公式的系数
    for(int j=0;j<Np;j++){
        CA1=cross(C_w,power(A_w,j+1));
        PHI_w.setblock(j*Nx,0,CA1);
        for (int k=0;k<Nc;k++){
            if(k<=j){
                CAB=cross(cross(C_w,power(A_w,j-k)),B_w);
                THETA_w.setblock(j*Nx,k*Nu,CAB);
            }
        }
    }

    //标准二次型形式约束项系数
    /*
    代价函数如下：
    J=(1/2)*X^T*(2*H)*X+f^T*X;
    H的形式如下：
    H<<THETA_w^T*Q*THETA+R        0
             0                                                        ROW
    f的形式如下：
    f<<2*E^T*Q*THETA       0
    */
    H1=R+cross(cross(THETA_w.transpose(),Q),THETA_w);
    H1*=2;
    H.setblock(0,0,H1);
    H[Nu*Nc][Nu*Nc]=Row;

    //按照公式还要减去Yref，但是此时Yref实际上是0，所以直接无视掉
    error=cross(PHI_w,kesi);
    Mf1=cross(cross(error.transpose(),Q),THETA_w);
    Mf1*=2;
    Mf.setblock(0,0,Mf1);
    Mf[0][Nu*Nc]=0;
    f=Mf.extractRow(0);

    //二次规划不等式约束输入项计算

    //A_t,下三角矩阵
    for(int i=0; i<Nc; i++){
        for(int j=0; j<Nc; j++){
            if(j<=i){
                A_t[i][j]=1;
            }
            else{
                A_t[i][j]=0;
            }
        }
    }

    A_I=kron(A_t,A_I1);

    Acons.setblock(0, 0, A_I);
	Acons.setblock(0, A_I.nrows(), Acons_Zero);
	A_I *= -1;
	Acons.setblock(A_I.ncols(), 0, A_I);
	Acons.setblock(A_I.ncols(), A_I.nrows(), Acons_Zero);
	CI0 = Acons.transpose();
	CI0 *= -1;
    //Acons<<A(30*30)   0(30*1)
    //          -A(30*30)   0(30*1)
    //CI0<<-A^T(60*60)    A^T(60*60)
    //             0(1*60)       0(1*60)
    //CI1_1<<1  -1  0  0 
    //                 0  0   1  -1
    //CI1<<CI1_1(2*4) 对角线上都是这个

	//CI-1<<1     -1        0      0
    //              0      0        1    -1
    //这个矩阵用于直接约束增量的变化量
	CI1_1[0][0] = 1;
	CI1_1[0][1] = -1;
	CI1_1[1][2] = 1;
	CI1_1[1][3] = -1;
	for(int i=0; i<Nc; i++){
		CI1.setblock(i*Nu, i*Nu*2, CI1_1);
	}

	CI.setblock(0, 0, CI0);
	CI.setblock(0, CI0.ncols(), CI1);

	Ut = kron(Ut_unit, U);
	umin[0][0] = vmin;
	umin[1][0] = wmin;
	umax[0][0] = vmax;
	umax[1][0] = wmax;
	Umin = kron(Ut_unit, umin);
	Umax = kron(Ut_unit, umax);

	ci0_0.setblock(0, 0, Umax-Ut);
	Umin *= -1;
	ci0_0.setblock(Nu*Nc, 0, Umin+Ut);
	ci0_m.setblock(0, 0, ci0_0);
	ci0_m.setblock(ci0_0.nrows(), 0, ci0_1);
	ci0 = ci0_m.extractColumn(0);

	double con = solve_quadprog(H, f, CI, ci0, X);
	//std::cout<<"guihua-X:"<<X<<std::endl;
	//Vector<double> u_piao(Nx_,Nu_);//Nx_,Nu_
	U[0][0] = kesi[3][0] + X[0];
	U[1][0] = kesi[4][0] + X[1];      
    //储存上一时刻的控制量（控制量增量），给下次循环使用

	Bcon.speed = U[0][0] + vd1;
	Bcon.angular_speed = U[1][0] + vd2;          //真实的控制量，用于返回发送给底盘
	return Bcon;
}

//回调函数，用于发布每一段路径点的参考速度和角速度
void mpc::velocityCallback(const nav_msgs::Odometry  &msg)
{
    control_aim.speed=0.6;
    control_aim.angular_speed=0.05;
    ROS_ERROR("aim_vx=%f",control_aim.speed);
    ROS_ERROR("aim_vw=%f",control_aim.angular_speed);
}

}
