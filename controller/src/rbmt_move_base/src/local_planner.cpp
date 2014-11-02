#include <rbmt_move_base/local_planner.h>

namespace rbmt_move_base {

LocalPlanner::LocalPlanner() {};
LocalPlanner::~LocalPlanner() {};

bool LocalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& global_plan, std::vector<geometry_msgs::Twist>* local_plan) {
  local_plan->clear();

  for (size_t i=0; i<(global_plan.size()-1); ++i) {
    geometry_msgs::PoseStamped waypoint_i, waypoint_j;
    waypoint_i = global_plan.at(i);
    waypoint_j = global_plan.at(i+1);

    geometry_msgs::Twist cmd_vel;

    //TODO @tttor: consider max vel, break into segments if one segment is too long
    cmd_vel.linear.x = waypoint_j.pose.position.x - waypoint_i.pose.position.x;
    cmd_vel.linear.y = waypoint_j.pose.position.y - waypoint_i.pose.position.y;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;// roll
    cmd_vel.angular.y = 0.0;// pitch
    cmd_vel.angular.z = tf::getYaw(waypoint_j.pose.orientation) - tf::getYaw(waypoint_i.pose.orientation);

    local_plan->push_back(cmd_vel);
  }

  return true;
}

}// namespace rbmt_move_base