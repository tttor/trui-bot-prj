#include <rbmt_move_base/global_planner.h>

namespace rbmt_move_base {

GlobalPlanner::GlobalPlanner() {
  ros::NodeHandle nh;

  const double max_linear_vel_default_val = 0.1;
  nh.param<double> ("max_linear_vel", max_linear_vel_, max_linear_vel_default_val);

};

GlobalPlanner::~GlobalPlanner() {};

bool GlobalPlanner::plan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal_pose, std::vector<geometry_msgs::PoseStamped>* global_plan) {
  using namespace std;

  Eigen::Vector2d s(start_pose.pose.position.x, start_pose.pose.position.y);
  Eigen::Vector2d g(goal_pose.pose.position.x, goal_pose.pose.position.y);

  // For now, consider only 2D pose
  const double dist = get_cartesian_dist<Eigen::Vector2d>(s, g);

  size_t n_segment = ((size_t) dist / max_linear_vel_);
  if (remainder(dist,max_linear_vel_) > 0.0) {
    ++n_segment;// since we desire each segment can be done in,at most, max_linear_vel_
  }

  Eigen::Vector2d segment;
  segment = (g - s) / n_segment;

  global_plan->clear();
  global_plan->push_back(start_pose);
  for (size_t i=0; i<n_segment; ++i) {
    Eigen::Vector2d w;
    w = s + ((i+1)*segment);// plus one as w is on the tip of i-th segment

    geometry_msgs::PoseStamped spose;
    spose.header = start_pose.header;// TODO fix me
    spose.pose.position.x = w(0);
    spose.pose.position.y = w(1);
    spose.pose.position.z = 0.0;
    spose.pose.orientation.x = 0.0;
    spose.pose.orientation.y = 0.0;
    spose.pose.orientation.z = 0.0;
    spose.pose.orientation.w = 1.0;

    global_plan->push_back(spose);
  }

  // for (size_t i=0; i<global_plan->size(); ++i) {
  //   geometry_msgs::PoseStamped spose;
  //   spose = global_plan->at(i);

  //   cout << "spose.pose.position.x= " << spose.pose.position.x << endl;
  // }

  return true;
}



}// namespace rbmt_move_base