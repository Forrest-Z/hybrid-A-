//#define _DEBUG__

#ifndef LMK_WS_SRC_A_STAR_INCLUDE_A_STAR_H_
#define LMK_WS_SRC_A_STAR_INCLUDE_A_STAR_H_
//if X has not been defined yet, compile the following below
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <queue>
#include <cmath>
#include <chrono>
#include <random>
#include <map>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

#include "ros/ros.h"
#include "nh_curve.h"

#include "nav_msgs/GetMap.h"
#include "map_proc/MapMetaP.h"
#include "tiguan_movebase.h"

namespace lmk_astar {
struct VehiclePose {
  double x_pose;
  double y_pose;
  double heading_angle;
  int x_index;
  int y_index;
  double cost_g;
  double total_cost;
  VehiclePose* parent;
  bool operator < (const VehiclePose& temp_node) const;
  bool operator > (const VehiclePose& temp_node) const;
  bool operator == (const VehiclePose& temp_node) const;
  bool operator != (const VehiclePose& temp_node) const;
  void operator = (const VehiclePose& temp_node);
  VehiclePose(double x, double y, double angle, double temp_cost_g, double temp_total_cost);
  VehiclePose(double x, double y, double angle, double temp_cost_g);
  VehiclePose(double x, double y, double angle);
  VehiclePose();
};
struct ROSMapData {
  int map_height;
  int map_length;
  float resolution;
  std::vector<std::vector<int>> map_occupancy;
  bool metadata_flag;
  bool mapdata_flag;
	ROSMapData();
};
class OpenList {
 public:
  OpenList();
  int find(VehiclePose search_pose) const;
  void pop();
  void insert(VehiclePose insert_pose);
  bool check_nearer(int index, VehiclePose check_pose);
  void decrease(int index, VehiclePose change_pose);
  bool empty();
  VehiclePose top();
 private:
  int heap_size_;
  const double cost_infinity_;
  void min_heapify(int index);
  std::vector<VehiclePose> openlist_data_;
};
class AStar {
 public:
  static const double pi_;
  AStar();
  virtual ~AStar() = default;
 protected:
  void acquire_mapdata();
  virtual double heuristic_func(VehiclePose cal_pose) = 0;
  virtual void update_neighbour(VehiclePose& cur_pose) = 0;
  virtual bool reach_destination(VehiclePose temp_pose) = 0;
  virtual bool collision_detection(VehiclePose check_pose) = 0;
  OpenList open_list_;
  ROSMapData map_data_;
  std::map<std::vector<int>, VehiclePose> closed_list_;
};
class HybridAStar : virtual public AStar {
 public:
  void Init();
  void Proc();
  ~HybridAStar();
  void hybrid_astar_search();
 protected:
  double heuristic_func(VehiclePose cal_pose);
  void update_neighbour(VehiclePose& cur_pose);
  bool reach_destination(VehiclePose temp_pose);
  bool collision_detection(VehiclePose check_pose);
 private:
  std::vector<std::vector<double>> motion_primitive(VehiclePose root_pose);
  void path_generator();
  void set_destination();
  void get_initial();
  std::vector<double> random_point(int pic_height, int pic_lenght);
  // funcitons that need to be precomputed, before the start of the on time planner
  void acquire_mapdata_fromjpg();
  void pre_compute_heuristic_cost();
  // functions that are used to draw demos
  void draw_demo();
  void draw_baseimg();
  // member parameters
  std::vector<VehiclePose> path_found_;
  std::vector<double> car_parameters_;
  tiguan_movebase::VehicleMoveBase tiguan_model_;
  VehiclePose initial_;
  VehiclePose destination_;
  std::vector<std::vector<double>> heuristic_lookup_talbe_;
};
class NormalAStar : virtual public AStar{
 public:
  void Init(VehiclePose desti);
  int normal_astar_search(VehiclePose initial_gird);
  std::vector<std::vector<int>> cost_map_;
 protected:
  double heuristic_func(VehiclePose start_gird);
  void update_neighbour(VehiclePose& cur_grid);
  bool reach_destination(VehiclePose temp_grid);
  bool collision_detection(VehiclePose check_grid);
 private:
  std::vector<std::vector<int>> motion_primitive(VehiclePose root_grid);
  int path_generator();
  VehiclePose destination_grid_;
};
} // namespace lmk_astar
// if X has already been defined, goto #else directly
#else
// however, nothing should be stated here
#endif //CATKIN_WS_SRC_A_STAR_INCLUDE_A_STAR_H_