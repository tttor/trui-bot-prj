#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace rbmt_nav {

class Navigator {
 public:
  Navigator(ros::NodeHandle nh);
  ~Navigator();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber cock_pose_sub_;

  void cock_pose_sub_cb (const geometry_msgs::PoseStampedConstPtr& msg);

  size_t send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac);
};

}// namespace rbmt_nav

#endif
