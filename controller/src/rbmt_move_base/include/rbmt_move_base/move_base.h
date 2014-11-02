#ifndef RBMT_MOVE_BASE_ACTION_H_
#define RBMT_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>
#include <iostream>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>

#include <rbmt_move_base/global_planner.h>
#include <rbmt_move_base/local_planner.h>

namespace rbmt_move_base {

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBase {
 public:
 	MoveBase(ros::NodeHandle nh, tf::TransformListener& tf_listener);
 	~MoveBase();

 private:
  void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
  geometry_msgs::PoseStamped get_pose();

  ros::NodeHandle nh_;
  MoveBaseActionServer* as_;
  GlobalPlanner* global_planner_;
  LocalPlanner* local_planner_;
  
  ros::Publisher vel_pub_;
  tf::TransformListener& tf_listener_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  double dt_realization_;
};

}// namespace rbmt_move_base
#endif 