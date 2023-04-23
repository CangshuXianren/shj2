#ifndef JINGO_VECTOR3_H
#define JINGO_VECTOR3_H


#include <pcl/point_types.h>



class Vector3 : public Eigen::Vector4d {
public:
  Vector3(double x, double y, double z)
      : Eigen::Vector4d(x, y, z, 0) {}

  Vector3(void)
      : Eigen::Vector4d(0, 0, 0, 0) {}

  template<typename OtherDerived>
  Vector3(const Eigen::MatrixBase <OtherDerived> &other)
      : Eigen::Vector4d(other) {}

  Vector3(const pcl::PointXYZI &p)
      : Eigen::Vector4d(p.x, p.y, p.z, 0) {}

  template<typename OtherDerived>
  Vector3 &operator=(const Eigen::MatrixBase <OtherDerived> &rhs) {
    this->Eigen::Vector4d::operator=(rhs);
    return *this;
  }

  Vector3 &operator=(const pcl::PointXYZ &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  Vector3 &operator=(const pcl::PointXYZI &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  double x() const { return (*this)(0); }

  double y() const { return (*this)(1); }

  double z() const { return (*this)(2); }

  double &x() { return (*this)(0); }

  double &y() { return (*this)(1); }

  double &z() { return (*this)(2); }

  // easy conversion
  operator pcl::PointXYZI() {
    pcl::PointXYZI dst;
    dst.x = x();
    dst.y = y();
    dst.z = z();
    dst.intensity = 0;
    return dst;
  }
};



#endif
