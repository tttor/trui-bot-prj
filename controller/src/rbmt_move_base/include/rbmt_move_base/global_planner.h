#ifndef RBMT_GLOBAL_PLANNER_H
#define RBMT_GLOBAL_PLANNER_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include <vector>
#include <iostream>
#include <cmath>

namespace rbmt_move_base {
class GlobalPlanner {
 public:
 	GlobalPlanner();
 	~GlobalPlanner();

  /*!
  Set the global_plan, i.e. trajectory, 
  for now, it merely consists of 2 waypoints: start and goal poses
  */
  bool plan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal_pose, std::vector<geometry_msgs::PoseStamped>* global_plan);

 private:
  template<typename T>
  double get_cartesian_dist(const T& p1, const T& p2) {
    const size_t n_dim = p1.size();

    double total = 0.0;
    for (size_t i=0; i<n_dim; ++i) {
      double d = p1(i) - p2(i);
      total += pow(d,2);
    }
      
    return sqrt(total);
  }
};

}// namespace rbmt_move_base

#endif