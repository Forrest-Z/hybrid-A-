#include "a_star.h"

// construction/deconstruction function of struct VehiclePose
lmk_astar::VehiclePose::VehiclePose() {
  parent = nullptr;
  cost_g = total_cost = -1.0;
  x_pose = y_pose = heading_angle = -1.0;
  x_index = y_index = -1;
}
lmk_astar::VehiclePose::VehiclePose(double x, double y, double angle, double temp_cost_g, double temp_total_cost) {
  x_pose = x;
  y_pose = y;
  x_index = static_cast<int>(x);
  y_index = static_cast<int>(y);
  heading_angle = angle;
  cost_g = temp_cost_g;
  parent = nullptr;
  total_cost = temp_total_cost;
}
lmk_astar::VehiclePose::VehiclePose(double x, double y, double angle, double temp_cost_g) {
  x_pose = x;
  y_pose = y;
  x_index = static_cast<int>(x);
  y_index = static_cast<int>(y);
  heading_angle = angle;
  cost_g = temp_cost_g;
  parent = nullptr;
  total_cost = -1.0;
}
lmk_astar::VehiclePose::VehiclePose(double x, double y, double angle) {
  x_pose = x;
  y_pose = y;
  x_index = static_cast<int>(x);
  y_index = static_cast<int>(y);
  heading_angle = angle;
  cost_g = total_cost = -1.0;
  parent = nullptr;
}
// note: another way to compare a struct is to define a "comparer" outside the struct
// and then use it as the input of functions/data_structures like: sort(), priority_queue
bool lmk_astar::VehiclePose::operator < (const VehiclePose& temp_node) const {
  return (total_cost < temp_node.total_cost);
}
bool lmk_astar::VehiclePose::operator > (const VehiclePose& temp_node) const {
  return (total_cost > temp_node.total_cost);
}
bool lmk_astar::VehiclePose::operator == (const VehiclePose& temp_node) const {
  if (x_index != temp_node.x_index || y_index != temp_node.y_index) {
    return false;
  } else {return true;}
}
bool lmk_astar::VehiclePose::operator != (const VehiclePose& temp_node) const {
  if (x_index == temp_node.x_index && y_index == temp_node.y_index) {
    return false;
  } else {return true;}
}
void lmk_astar::VehiclePose::operator = (const VehiclePose& temp_node) {
  parent = temp_node.parent;
  x_pose = temp_node.x_pose;
  y_pose = temp_node.y_pose;
  x_index = temp_node.x_index;
  y_index = temp_node.y_index;
  heading_angle = temp_node.heading_angle;
  cost_g = temp_node.cost_g;
  total_cost = temp_node.total_cost;
}
// construction/deconstruction function of struct ROSMapData
lmk_astar::ROSMapData::ROSMapData():metadata_flag(false),mapdata_flag(false) {
  map_height = map_length = -1;
}
// functions in class OpenList
// binary heap
lmk_astar::OpenList::OpenList():heap_size_(0), cost_infinity_(5000.0) {};
void lmk_astar::OpenList::min_heapify(int index) {
  int left((index+1)*2),right((index+1)*2+1), smallest(index+1);
  VehiclePose swap_temp;
  while (index < heap_size_) {
    left = (index+1)*2;
    right = (index+1)*2+1;
    if (left <= heap_size_ && openlist_data_[left-1] < openlist_data_[index]) {
      smallest = left;
    } else if (right <= heap_size_ && openlist_data_[right-1] < openlist_data_[smallest-1]) {
      smallest = right;
    } else {}
    if ((index+1) != smallest) {
      swap_temp = openlist_data_[index];
      openlist_data_[index] = openlist_data_[smallest-1];
      openlist_data_[smallest-1] = swap_temp;
      index = smallest-1;
    } else {break;}
  }
};
// if -1 returned, target pose is not found
inline int lmk_astar::OpenList::find(VehiclePose search_pose) const {
  int size = openlist_data_.size();
  for (int i = 0; i < size; ++i) {
    if (search_pose == openlist_data_[i]) {
      return i;
    }
  }
  return -1;
}
void lmk_astar::OpenList::insert(VehiclePose insert_pose) {
  heap_size_ = heap_size_ + 1;
  VehiclePose temp_pose(-1.0, -1.0, -1.0, cost_infinity_, cost_infinity_);
  if (heap_size_ > openlist_data_.size()) {
    openlist_data_.push_back(temp_pose);
  } else {
    openlist_data_[heap_size_-1] = temp_pose;
  }
  decrease(heap_size_-1, insert_pose);
}
// keywords "inline" should be put here, it will not work if placed in the .h file, function statement
inline bool lmk_astar::OpenList::check_nearer(int index, VehiclePose check_pose) {
  if (index < 0 || index > (heap_size_-1)) {
    ROS_ERROR("error: invalid index, could not compare");
    return false;
  }
  if (check_pose.cost_g < openlist_data_[index].cost_g) {
    return true;
  } else {return false;}
}
inline void lmk_astar::OpenList::pop() {
  if (heap_size_ < 1) {
    ROS_ERROR("error: empty openlist, could not pop");
    return;
  }
  openlist_data_[0] = openlist_data_[heap_size_-1];
  //openlist_data_.pop_back();
  heap_size_ = heap_size_ - 1;
  min_heapify(0);
}
void lmk_astar::OpenList::decrease(int index, VehiclePose change_pose) {
  openlist_data_[index] = change_pose;
  VehiclePose temp_pose;
  while (index > 0 && openlist_data_[(index+1)/2-1] > openlist_data_[index]) {
    temp_pose = openlist_data_[index];
    openlist_data_[index] = openlist_data_[(index+1)/2-1];
    openlist_data_[(index+1)/2-1] = temp_pose;
    index = (index+1)/2-1;
  }
}
bool lmk_astar::OpenList::empty() {
  return ((heap_size_ > 0) ? false:true);
}
inline lmk_astar::VehiclePose lmk_astar::OpenList::top() {
  if (openlist_data_.empty()) {
    ROS_ERROR("error: empty openlist, random pose returned");
    VehiclePose temp;
    return temp;
  }
  return openlist_data_[0];
}
// static function, in case std::sort() will be used
bool lmk_astar::OpenList::sort_cmp(VehiclePose pose1, VehiclePose pose2) {
  return (pose1.total_cost > pose2.total_cost);
}
// construction/deconstruction function of class AStar
lmk_astar::AStar::AStar():astar_nh_("astar_planner"),pi_(3.14159) {
  //google::InitGoogleLogging("astar_planning");
#ifdef _DEBUG__
  std::cout << "Debug mode" << std::endl;
#endif
  acquire_mapdata();
  set_destination();
  get_initial();
  //draw_baseimg();
  astar_search();
  //LOG(INFO) << "glog set successfully" << std::endl;
}
lmk_astar::AStar::~AStar() {}
// public member function
// main logic
void lmk_astar::AStar::astar_search() {
  VehiclePose temp_pose(initial_.x_pose, initial_.y_pose, initial_.heading_angle, 0);
  temp_pose.total_cost = temp_pose.cost_g + heuristic_func(temp_pose);
  open_list_.insert(temp_pose);
  while (!open_list_.empty()) {
    temp_pose = open_list_.top();
    open_list_.pop();
    // insert into closed list
    map_data_.map_occupancy[temp_pose.y_index][temp_pose.x_index] = 2;
    closed_list_.insert(std::pair<std::vector<int>, VehiclePose>({temp_pose.x_index, temp_pose.y_index}, temp_pose));
    if (reach_destination(temp_pose)) {     
      destination_.parent = &(closed_list_.find({temp_pose.x_index, temp_pose.y_index})->second);
      closed_list_.insert(std::pair<std::vector<int>, VehiclePose>({destination_.x_index, destination_.y_index}, destination_));
      path_generator();
      draw_demo();
      return;
    } else {update_neighbour(temp_pose);}
  }
  ROS_INFO("no path found");
}
// private member function
std::vector<std::vector<double>> lmk_astar::AStar::motion_primitive(VehiclePose root_pose) {
  std::vector<std::vector<double>> neighbour_nodes;
  /*neighbour_nodes = {{root_pose.y_pose-1, root_pose.x_pose, root_pose.heading_angle, 1}, {root_pose.y_pose+1, root_pose.x_pose, root_pose.heading_angle, 1},
  {root_pose.y_pose, root_pose.x_pose-1, root_pose.heading_angle, 1}, {root_pose.y_pose, root_pose.x_pose+1, root_pose.heading_angle, 1},
  {root_pose.y_pose-1, root_pose.x_pose-1, root_pose.heading_angle, 1.414}, {root_pose.y_pose+.414, root_pose.x_pose-.414, root_pose.heading_angle, 1.414},
  {root_pose.y_pose-1, root_pose.x_pose+1, root_pose.heading_angle, 1.414}, {root_pose.y_pose+1, root_pose.x_pose+1, root_pose.heading_angle, 1.414}};*/
  VehiclePose temp_pose;
  double resolution = map_data_.resolution;
  std::vector<double> root_pose_vector = {root_pose.y_pose*resolution, root_pose.x_pose*resolution, root_pose.heading_angle};
  std::vector<double> temp_storage;
  double total_angle = car_parameters_[2];
  double temp_angle(0.0);
  for (int i = 0; i < 5; ++i) {
    temp_angle = -total_angle + (total_angle/2.0)*i;
    temp_storage = tiguan_model_.tiguan_bicycle_module(8.0, temp_angle, 0.08, root_pose_vector);
    temp_storage[0] = temp_storage[0]/resolution;
    temp_storage[1] = temp_storage[1]/resolution;
    temp_storage[3] = temp_storage[3]/resolution;
    neighbour_nodes.push_back(temp_storage);
  }
#ifdef _DEBUG__
  std::cout << "root node x is " << root_pose.x_pose << " y is " << root_pose.y_pose << " angle is " << root_pose.heading_angle << std::endl;
  for (int j = 0; j < 5; ++j) {
    temp_storage = neighbour_nodes[j];
    std::cout << "new motion x is " << temp_storage[1] << " y is " << temp_storage[0] << " angle is " << temp_storage[2] << " path length is " << temp_storage[3] << std::endl;
  }
#endif
  return neighbour_nodes;
}
void lmk_astar::AStar::path_generator() {
  VehiclePose* route_pointer(&closed_list_.find({destination_.x_index, destination_.y_index})->second);
  while (route_pointer->parent != nullptr) {
    path_found_.push_back(*route_pointer);
    route_pointer = route_pointer->parent;
  }
}
double lmk_astar::AStar::heuristic_func(VehiclePose cal_pose) {
  if (destination_.y_pose == -1 || destination_.x_pose == -1) {
    ROS_ERROR("error: no destination set, could not calculate heuristic");
    return 0;
  } else {return std::sqrt(std::pow((cal_pose.x_pose - destination_.x_pose), 2)+std::pow((cal_pose.y_pose - destination_.y_pose), 2));}
}
// got able to run, but result not shown yet
void lmk_astar::AStar::update_neighbour(VehiclePose& cur_pose) {
  std::vector<std::vector<double>> neighbour_nodes = motion_primitive(cur_pose);
  VehiclePose temp_pose;
  int open_index(0);
  for (auto i : neighbour_nodes) {
    temp_pose = {i[1], i[0], i[2]};
    // check if the pose is valid: valid index and collision free
    if (i[1] >=0 && i[1] <= (map_data_.map_length-1) && i[0] >=0 && i[0] <= (map_data_.map_height-1) && collision_detection(temp_pose)) {
      // if in closed list, do nothing
      if (map_data_.map_occupancy[temp_pose.y_index][temp_pose.x_index] == 2) {continue;}
      // update cost_g
      temp_pose.cost_g = cur_pose.cost_g + i[3];
      // calculate the heuristic distance and update the current total_cost
      temp_pose.total_cost = heuristic_func(temp_pose) + temp_pose.cost_g;
      temp_pose.parent = &(closed_list_.find({cur_pose.x_index, cur_pose.y_index})->second);
      // if in openlist: check if its cost_g nearer
      if (map_data_.map_occupancy[temp_pose.y_index][temp_pose.x_index] == 3) {
        open_index = open_list_.find(temp_pose);
        if (open_list_.check_nearer(open_index, temp_pose)) {
          open_list_.decrease(open_index, temp_pose);
        } else {}
      } else {
        open_list_.insert(temp_pose);
      }
    }
  }
}
void lmk_astar::AStar::acquire_mapdata() {
  int size(0);
  while(!map_data_.metadata_flag) {
    ros::ServiceClient map_meta_client = astar_nh_.serviceClient<map_proc::MapMetaP>("/map_metadata_planner");
    map_proc::MapMetaP map_meta;
    if (map_meta_client.call(map_meta)) {
      map_data_.map_length = map_meta.response.length;
      map_data_.map_height = map_meta.response.height;
      map_data_.resolution = map_meta.response.resolution;
    } else {ROS_ERROR("error: failed to call map metadata");}
    if (map_data_.map_height != -1 && map_data_.map_length != -1) {
      map_data_.metadata_flag = true;
      ROS_INFO("map data successfully received");
    } else {map_data_.metadata_flag = false;}
  }
  std::vector<std::vector<int>> map_copy(map_data_.map_height, std::vector<int>(map_data_.map_length, -1));
  ros::ServiceClient map_client = astar_nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap map_rawdata;
  while(!map_data_.mapdata_flag) {
    if (map_client.call(map_rawdata)) {
      size = (map_rawdata.response.map.data).size();
      map_data_.mapdata_flag = true;
      for (int i = 0; i < size; ++i) {
        int x_index = i%map_data_.map_length;
        int y_index = i/map_data_.map_length;
        if (map_rawdata.response.map.data[i] == -1) {
          map_copy[y_index][x_index] = 1;
        } else if (map_rawdata.response.map.data[i] == 0) {
          map_copy[y_index][x_index] = 0;
        } else if (map_rawdata.response.map.data[i] == 100) {
          map_copy[y_index][x_index] = 1;
        } else {}
      }
    } else {ROS_ERROR("error: falided to receive map data");}
    map_data_.map_occupancy = map_copy;
  }
  car_parameters_ = tiguan_model_.get_parameter();
}
bool lmk_astar::AStar::reach_destination(VehiclePose temp_pose) {
  if (destination_.x_index == -1 || destination_.y_index == -1) {
    ROS_ERROR("error: no destination set");
    return false;
  } else if (5 >= std::sqrt(std::pow((temp_pose.y_pose - destination_.y_pose), 2) + std::pow((temp_pose.x_pose - destination_.x_pose), 2))) {
    return true;
  } else {return false;}
}
bool lmk_astar::AStar::collision_detection(VehiclePose check_pose) {
  if (map_data_.map_occupancy[check_pose.y_index][check_pose.x_index] == 1) {
    return false;
  }
  return true;
}
void lmk_astar:: AStar::set_destination() {
  std::vector<double> temp_config(3, 0.0);
  VehiclePose temp_pose;
  temp_config = random_point(map_data_.map_height, map_data_.map_length);
  temp_pose = {temp_config[1], temp_config[0], temp_config[2]};
  while(!collision_detection(temp_pose)){
    temp_config = random_point(map_data_.map_height, map_data_.map_length);
    temp_pose = {temp_config[1], temp_config[0], temp_config[2]};
  }
  destination_ = temp_pose;
}
void lmk_astar::AStar::get_initial() {
  std::vector<double> temp_config(3, 0.0);
  VehiclePose temp_pose;
  temp_config = random_point(map_data_.map_height, map_data_.map_length);
  temp_pose = {temp_config[1], temp_config[0], temp_config[2], 0};
  while (!collision_detection(temp_pose)) {
    temp_config = random_point(map_data_.map_height, map_data_.map_length);
    temp_pose = {temp_config[1], temp_config[0], temp_config[2], 0};
  }
  initial_ = temp_pose;
}
std::vector<double> lmk_astar::AStar::random_point(int pic_height, int pic_length) {
  std::vector<double> random_config(3, 0.0);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution_x_axis(0, pic_length);
  std::uniform_real_distribution<double> distribution_y_axis(0, pic_height);
  std::uniform_real_distribution<double> distribution_angle(-pi_, pi_);
  random_config[0] = distribution_y_axis(generator);
  random_config[1] = distribution_x_axis(generator);
  random_config[2] = distribution_angle(generator);
  return random_config;
}
void lmk_astar::AStar::draw_demo() {
  ROS_INFO("start drawing pic");
  cv::Mat show_img = cv::imread("/home/mingkun/lmk_ws/src/a_star/assets/map_base_img.jpg");
  cv::Point temp_point, end_point;
  VehiclePose* route_pointer(&(closed_list_.find({destination_.x_index, destination_.y_index})->second));
  // draw destination point and initial point
  temp_point.x = destination_.x_index;
  temp_point.y = destination_.y_index;
  end_point.x = temp_point.x + 20*std::cos(destination_.heading_angle);
  end_point.y = temp_point.y + 20*std::sin(destination_.heading_angle);
  cv::circle(show_img, temp_point, 8, cv::Scalar(0, 0, 255), 2);
  cv::line(show_img, temp_point, end_point, cv::Scalar(0, 0, 255), 2);
  temp_point.x = initial_.x_index;
  temp_point.y = initial_.y_index;
  end_point.x = temp_point.x + 20*std::cos(initial_.heading_angle);
  end_point.y = temp_point.y + 20*std::sin(initial_.heading_angle);
  cv::circle(show_img, temp_point, 8, cv::Scalar(255, 0, 0), 2);
  cv::line(show_img, temp_point, end_point, cv::Scalar(255, 0, 0), 2);
  for (int i = 0; i < map_data_.map_height; ++i) {
    for (int j = 0; j < map_data_.map_length; ++j) {
      if (map_data_.map_occupancy[i][j] == 2) {
        show_img.at<cv::Vec3b>(i,j)[0] = 0;
        show_img.at<cv::Vec3b>(i,j)[1] = 255;
        show_img.at<cv::Vec3b>(i,j)[2] = 0;
      }
    }
  }
  while (route_pointer->parent != nullptr) {
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[0] = 0;
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[1] = 0;
    show_img.at<cv::Vec3b>(route_pointer->y_index, route_pointer->x_index)[2] = 255;
    route_pointer = route_pointer->parent;
  }
  cv::imwrite("/home/mingkun/lmk_ws/src/a_star/assets/test1.jpg", show_img);
}
void lmk_astar::AStar::draw_baseimg() {
  cv::Mat img_yutian = cv::Mat::zeros(map_data_.map_height, map_data_.map_length, CV_8UC3);
  for (int i = 0; i < map_data_.map_height; ++i) {
   uchar* image_pointer = img_yutian.ptr(i);
    for (int j = 0; j < map_data_.map_length; ++j) {
      uchar* pixel_pointer = image_pointer;
      if (map_data_.map_occupancy[i][j] == 1) {
        img_yutian.at<cv::Vec3b>(i, j)[0] = 255;
        img_yutian.at<cv::Vec3b>(i, j)[1] = 255;
        img_yutian.at<cv::Vec3b>(i, j)[2] = 255;
      } else if (map_data_.map_occupancy[i][j] == 0) {
        continue;
      } else {
        ROS_ERROR("error: wrong point got when drawing baseimg");
      }
    }
  }
  cv::imwrite("/home/mingkun/lmk_ws/src/a_star/assets/map_base_img.jpg", img_yutian);
}