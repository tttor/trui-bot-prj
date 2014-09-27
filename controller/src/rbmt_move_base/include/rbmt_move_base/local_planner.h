#ifndef RBMT_LOCAL_PLANNER_H
#define RBMT_LOCAL_PLANNER_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <iostream>

namespace rbmt_move_base {

class LocalPlanner {
 public:
 	LocalPlanner();
 	~LocalPlanner();

  /*!
	Outputs sequence of velocity (cmd_vel).
	Assume an obstacle-free workspace: consequently, a straigh-path suffices at all times.
	Assume a constant unit time between two waypoints.
  */
  bool plan(const std::vector<geometry_msgs::PoseStamped>& global_plan, std::vector<geometry_msgs::Twist>* local_plan);
 private:
};

}// namespace rbmt_move_base

#endif