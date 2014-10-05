#include <rbmt_move_base/global_planner.h>

namespace rbmt_move_base {

GlobalPlanner::GlobalPlanner() {};
GlobalPlanner::~GlobalPlanner() {};

bool GlobalPlanner::plan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal_pose, std::vector<geometry_msgs::PoseStamped>* global_plan) {
	global_plan->clear();
  using namespace std;

  global_plan->push_back(start_pose);
  global_plan->push_back(goal_pose);

  // for (size_t i=0; i<global_plan->size(); ++i) {
  //   geometry_msgs::PoseStamped spose;
  //   spose = global_plan->at(i);

  //   cout << "spose.pose.position.x= " << spose.pose.position.x << endl;
  // }

  return true;
}

}// namespace rbmt_move_base