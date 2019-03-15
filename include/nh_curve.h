#ifndef LMK_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_
#define LMK_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_

#include <vector>
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>

#include "tiguan_movebase.h"

namespace lmk_libraries {
class NHCurve {
 public:
  void RSCurve();
  void DBCurve();
 private:
  double desired_velocity_;
  int curve_type_;
};
}
#endif //CATKIN_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_