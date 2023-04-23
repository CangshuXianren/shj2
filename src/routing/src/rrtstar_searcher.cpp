#include <rrt_searcher/rrtstar_searcher.h>

using namespace std;
using namespace Eigen;

void RRTstarPreparatory::initGridMap(double _resolution, Vector2d global_xy_l, Vector2d global_xy_u, int max_x_id, int max_y_id)
{   
    gl_xl = global_xy_l(0);
    gl_yl = global_xy_l(1);

    gl_xu = global_xy_u(0);
    gl_yu = global_xy_u(1);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
}

void RRTstarPreparatory::setObs(const double coord_x, const double coord_y)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl   || coord_x >= gl_xu || coord_y >= gl_yu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    
    data[idx_x * GLY_SIZE + idx_y] = 1;
}

bool RRTstarPreparatory::checkstate(const double coord_x, const double coord_y)
{
    Vector2d pt;
    Vector2i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && (data[idx_x * GLY_SIZE + idx_y] < 1));
}

bool RRTstarPreparatory::isObsFree(const double coord_x1, const double coord_y1, const double coord_x2, const double coord_y2)
{
    Vector2d pt1, pt2;
    Vector2i idx1, idx2;
    
    pt1(0) = coord_x1;
    pt1(1) = coord_y1;
    // idx1 = coord2gridIndex(pt1);
    pt2(0) = coord_x2;
    pt2(1) = coord_y2;
    // idx2 = coord2gridIndex(pt2);

    // int idx_x1 = idx1(0);
    // int idx_y1 = idx1(1);
    // int idx_x2 = idx2(0);
    // int idx_y2 = idx2(1);

    double k = (coord_y2 - coord_y1) / (coord_x2 - coord_x1);
    double tempx = coord_x1;
    double tempy = coord_y1;
    
    for(int i = 1; sqrt(pow((tempx - coord_x2), 2) + pow((tempx - coord_x2), 2)) > resolution; ++i)
    {
        tempx = coord_x1 + i * copysign(resolution, k) ;
        tempy = coord_y1 + i * k * resolution;
        if(!checkstate(tempx, tempy))
        {
            return false;
        }
    }
    return true;
}

Vector2d RRTstarPreparatory::gridIndex2coord(const Vector2i & index) 
{
    Vector2d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;

    return pt;
}

Vector2i RRTstarPreparatory::coord2gridIndex(const Vector2d & pt) 
{
    Vector2i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector2d RRTstarPreparatory::coordRounding(const Eigen::Vector2d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}
