#include "hybrid_a_star.h"
#include "nh_curve.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "astar_planner");
  ros::NodeHandle freespace_plan_nh;
  lmk_astar::AStar hybrid_astar;
  //ros::spin();
  return 0;
}