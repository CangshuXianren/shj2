#ifndef JINGO_TWIST_H
#define JINGO_TWIST_H


#include "Angle.h"
#include "Vector3.h"


class Twist {
public:
  Twist()
        : rot_x(),
          rot_y(),
          rot_z(),
          pos() {};

  Angle rot_x;
  Angle rot_y;
  Angle rot_z;
  Vector3 pos;
};


#endif
