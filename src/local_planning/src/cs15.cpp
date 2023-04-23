#include "cs15.h"

CSRefPath& CSRefPath::operator=(CSRefPath& input) {
    std::vector<double> ().swap(r_curvature);
    std::vector<double> ().swap(r_s);
    std::vector<double> ().swap(r_x);
    std::vector<double> ().swap(r_y);
    std::vector<double> ().swap(r_yaw);
    this->r_curvature = input.r_curvature;
    this->r_s = input.r_s;
    this->r_x = input.r_x;
    this->r_y = input.r_y;
    this->r_yaw = input.r_yaw;
    return *this;
}

void CSRefPath::joint(CSRefPath &path){
    double original_length = r_s.back();
    std::cout << "[cs_planner]: pre_joint_s = " << r_s.back() << std::endl;
    std::cout << "[cs_planner]: new_seg_s = " << path.r_s.back() << std::endl;
    for(int i = 1; i < path.r_curvature.size(); ++i){
        r_curvature.emplace_back(path.r_curvature[i]);
        r_x.emplace_back(path.r_x[i]);
        r_y.emplace_back(path.r_y[i]);
        r_yaw.emplace_back(path.r_yaw[i]);
        r_s.emplace_back(path.r_s[i] + original_length);
    }
    std::cout << "[cs_planner]: post_joint_s = " << r_s.back() << std::endl;
}

void CSRefPath::copypath(CSRefPath &path, int index){
    double initial_length = path.r_s[index];
    for(int i = index; i < path.r_curvature.size(); ++i){
        r_curvature.emplace_back(path.r_curvature[i]);
        r_x.emplace_back(path.r_x[i]);
        r_y.emplace_back(path.r_y[i]);
        r_yaw.emplace_back(path.r_yaw[i]);
        r_s.emplace_back(path.r_s[i] - initial_length);
        std::cout << "[cs_planner]: tmp_x = " << path.r_x[i] << ",tmp_y = " << path.r_y[i] << std::endl;
        std::cout << "[cs_planner]: tmp_s = " << path.r_s[i] << "[" << i << "]" << std::endl;
    }
    std::cout << "[cs_planner]: match_point_s = " << initial_length << "[" << index << "]" << std::endl;
    std::cout << "[cs_planner]: last_total_length = " << path.r_s.back() << ",now_total_length = " << r_s.back() << std::endl;
}

void CSRefPath::CSclear(){
    r_curvature.clear();
    r_s.clear();
    r_x.clear();
    r_y.clear();
    r_yaw.clear();
}

void CSRefPath::load(Spline2D &spline, double overall_length){
    double refspline_length = spline.s.back();
    for(double i = 0.0; i <= refspline_length; i += overall_length / 4){
        std::array<double, 2> point_ = spline.calc_postion(i);
        r_x.emplace_back(point_[0]);
        r_y.emplace_back(point_[1]);
        r_yaw.emplace_back(spline.calc_yaw(i));
        r_curvature.emplace_back(spline.calc_curvature(i));
        r_s.emplace_back(i);
    }
    std::array<double,2> point_=spline.calc_postion(refspline_length);
    r_x.emplace_back(point_[0]);
    r_y.emplace_back(point_[1]);
    r_yaw.emplace_back(spline.calc_yaw(refspline_length));
    r_curvature.emplace_back(spline.calc_curvature(refspline_length));
    r_s.emplace_back(refspline_length);
}

std::vector<double> vec_diff(const std::vector<double> &input)
{
    std::vector<double> output;
    for(unsigned int i=01;i<input.size();i++)
    {
        output.emplace_back(input[i]-input[i-1]);
    }
    return output;
}

std::vector<double> cum_sum(const std::vector<double> &input)
{
    std::vector<double> output;
    double temp = 0;
    for(unsigned int i=0; i<input.size(); i++)
    {
        temp += input[i];
        output.emplace_back(temp);
    }
    return output;
}

double Spline::calc(double t)
{
    if(t<x.front() || t>x.back())
    {
        throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
};

double Spline::calc_d(double t)
{
    if(t<x.front() || t>x.back())
    {
        throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx-1);
    double dx = t - x[seg_id];
    return b[seg_id]  + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
}

double Spline::calc_dd(double t)
{
    if(t<x.front() || t>x.back())
    {
        throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
}

Eigen::MatrixXd Spline::calc_A()
{
    Eigen::MatrixXd A=Eigen::MatrixXd::Zero(nx,nx);
    /*自由边界，即第一段第一个点和最后一点最后一个点的二阶导数为0
    */
        A(0, 0) = 1;
        for(int i=0; i<nx-1; i++)
        {
            if (i != nx-2)
            {
                A(i+1, i+1) = 2 * (h[i] + h[i+1]);
            }
            A(i+1, i) = h[i];
            A(i, i+1) = h[i];
        }
        A(0, 1) = 0.0;
        A(nx-1, nx-2) = 0.0;
        A(nx-1, nx-1) = 1.0;
        return A;
    /*受约束边界，即第一段第一个点和最后一点最后一个点的一阶导数为给定值
    
    */
}

Eigen::VectorXd Spline::calc_B()
{
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for(int i=0; i<nx-2; i++)
    {
        B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
    }
    return B;
}

int Spline::bisect(double t, int start, int end)
{
    int mid = (start+end)/2;
    if (t==x[mid] || end-start<=1)
    {
        return mid;
    }
    else if (t>x[mid])
    {
        return bisect(t, mid, end);
    }
    else
    {
        return bisect(t, start, mid);
    }
}

std::array<double,2> Spline2D::calc_postion(double s_t)
{
    double x = sx.calc(s_t);
    double y = sy.calc(s_t);
    return {{x, y}};
};

double Spline2D::calc_curvature(double s_t)
{
    double dx = sx.calc_d(s_t);
    double ddx = sx.calc_dd(s_t);
    double dy = sy.calc_d(s_t);
    double ddy = sy.calc_dd(s_t);

    // cy
    return (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 1.5);
};

double Spline2D::calc_yaw(double s_t)
{
    double dx = sx.calc_d(s_t);
    double dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
};

std::vector<double> Spline2D::calc_s(std::vector<double> x, std::vector<double> y)
{
    std::vector<double> ds;
    std::vector<double> out_s{0};
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);
    for(unsigned int i=0; i<dx.size(); i++)
    {
        ds.emplace_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
    }
    std::vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
}

void CScore15::UpdateKeeper() {
    keeper[0].x = mystate.x;
    keeper[0].y = mystate.y;
    keeper[0].yaw = mystate.yaw;
    keeper[1].x = front.x;
    keeper[1].y = front.y;
    keeper[1].yaw = front.yaw;
}

int CScore15::findMatchpoint(double x, double y, CSRefPath& input_trajectory){
    int num = input_trajectory.r_curvature.size();
    double dis_min = std::numeric_limits<double>::max();
    int index = 0;
    for(int i=0; i < num; i++)
    {
        double temp_dis = std::pow(input_trajectory.r_x[i] - x, 2) + std::pow(input_trajectory.r_y[i] - y, 2);
        //ROS_ERROR("%s,%f.%s,%f","r_x",r_x[i],"r_y",r_y[i]);
        //ROS_ERROR("%s,%f","distance",temp_dis);
        if(temp_dis < dis_min)
        {
            dis_min = temp_dis;
            index = i;
        }
    }
    // ROS_ERROR("%s,%i","nearest point",index);
    return index;
}
