#ifndef LMK_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_
#define LMK_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_

#include <vector>
#include <algorithm>
#include <vector>
#include <iostream>

namespace lmk_libraries {
class NHCurve {
 public:
  void RSCurve();
  void DBCurve();
 private:
  int k;
};
}
#endif //CATKIN_WS_SRC_A_STAR_INCLUDE_NH_CURVE_H_