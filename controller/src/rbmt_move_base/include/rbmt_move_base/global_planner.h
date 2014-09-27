#ifndef RBMT_GLOBAL_PLANNER_H
#define RBMT_GLOBAL_PLANNER_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <iostream>

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
};

}// namespace rbmt_move_base

#endif