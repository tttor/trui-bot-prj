#include <rbmt_move_base/move_base.h>
#include <geometry_msgs/Twist.h>

namespace rbmt_move_base {

MoveBase::MoveBase(): as_(NULL) {
  using namespace std;

	as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
	as_->start();
  ROS_INFO("move_base: up and running");
}

MoveBase::~MoveBase() { 
  if(as_ != NULL)
      delete as_;
}

void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal) {
  using namespace std;
  ROS_INFO("move_base: executeCb");

  // TODO @tttor: do planning, then publish velocity to the cmd_vel topic
  cout < "MoveBase::executeCb\n";

  as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");  
}

}// namespace rbmt_move_base