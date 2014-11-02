#include <rbmt_move_base/global_planner.h>

namespace rbmt_move_base {

GlobalPlanner::GlobalPlanner() {
  ros::NodeHandle nh;

  const double max_linear_vel_default_val = 0.1;
  nh.param<double> ("max_linear_vel", max_linear_vel_, max_linear_vel_default_val);

  const double max_angular_vel_default_val = 0.25*M_PI;  
  nh.param<double> ("max_angular_vel", max_angular_vel_, max_angular_vel_default_val);

};

GlobalPlanner::~GlobalPlanner() {};

bool GlobalPlanner::plan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal_pose, std::vector<geometry_msgs::PoseStamped>* global_plan) {
  using namespace std;

  //
  Eigen::Vector2d s(start_pose.pose.position.x, start_pose.pose.position.y);
  Eigen::Vector2d g(goal_pose.pose.position.x, goal_pose.pose.position.y);
  double s_yaw = tf::getYaw(start_pose.pose.orientation);
  double g_yaw = tf::getYaw(goal_pose.pose.orientation);

  // 
  const double linear_dist = get_cartesian_dist<Eigen::Vector2d>(s, g);
  const double angular_dist = get_yaw_dist(g_yaw, s_yaw);

  size_t n_segment = ((size_t) linear_dist / max_linear_vel_);
  if (remainder(linear_dist,max_linear_vel_) > 0.0) {
    ++n_segment;// since we desire each segment can be done in, at most, max_linear_vel_
  }

  if ((std::abs(angular_dist)/n_segment) > max_angular_vel_) {
    n_segment = ((size_t) angular_dist / max_angular_vel_);
    if (remainder(angular_dist,max_angular_vel_) > 0.0) {
      ++n_segment;// since we desire each segment can be done in, at most, max_angular_vel_
    }    
  }

  //
  Eigen::Vector2d segment;
  segment = (g - s) / n_segment;

  double angular_segment;
  angular_segment = angular_dist / n_segment;

  //
  global_plan->clear();
  global_plan->push_back(start_pose);
  for (size_t i=0; i<n_segment; ++i) {
    size_t mul = i+1;// plus one as w is on the tip of i-th segment

    Eigen::Vector2d w;
    w = s + (mul*segment);

    double angular_w;
    angular_w = s_yaw + (mul*angular_segment);

    geometry_msgs::PoseStamped spose;
    spose.header = start_pose.header;// TODO fix me
    spose.pose.position.x = w(0);
    spose.pose.position.y = w(1);
    spose.pose.position.z = 0.0;
    spose.pose.orientation = tf::createQuaternionMsgFromYaw(angular_w);

    global_plan->push_back(spose);
  }

  return true;
}

double GlobalPlanner::get_yaw_dist(const double& y2, const double& y1) {
  double dist = y2 - y1;

  if (dist > M_PI) return -1.0 * (y1 + (2*M_PI-y2));// multiplied by -1.0 for CW rotation
  else return dist;
}

}// namespace rbmt_move_base