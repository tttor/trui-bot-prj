#include <rbmt_move_base/local_planner.h>

namespace rbmt_move_base {

LocalPlanner::LocalPlanner() {};
LocalPlanner::~LocalPlanner() {};

bool LocalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& global_plan, std::vector<geometry_msgs::Twist>* local_plan) {
  const double dt = 1.0;// in seconds

  local_plan->clear();
  for (size_t i=0; i<(global_plan.size()-1); ++i) {
    //
    geometry_msgs::PoseStamped waypoint_i, waypoint_j;
    waypoint_i = global_plan.at(i);
    waypoint_j = global_plan.at(i+1);

    double dx = waypoint_j.pose.position.x - waypoint_i.pose.position.x;
    double dy = waypoint_j.pose.position.y - waypoint_i.pose.position.y;
    double dtheta = tf::getYaw(waypoint_j.pose.orientation) - tf::getYaw(waypoint_i.pose.orientation);

    //
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = dx/dt;
    cmd_vel.linear.y = dy/dt;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;// roll
    cmd_vel.angular.y = 0.0;// pitch
    cmd_vel.angular.z = dtheta/dt;

    //
    local_plan->push_back(cmd_vel);
  }

  return true;
}

}// namespace rbmt_move_base