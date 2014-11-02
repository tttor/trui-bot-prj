#include <rbmt_nav/navigator.h>

namespace rbmt_nav {

Navigator::Navigator(ros::NodeHandle nh): nh_(nh) {
  cock_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("cock_pose", 1, &Navigator::cock_pose_sub_cb, this);
}

Navigator::~Navigator() {

}

void Navigator::cock_pose_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg) {
    //
  const std::string action_server_name = "move_base";
  MoveBaseClient ac(action_server_name);
  ac.waitForServer();

  //
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = *msg;

  send_goal(goal, ac);
}

size_t Navigator::send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac) {
  const bool debug = !false;

  ROS_INFO_COND(debug, "send_goal(): BEGIN");
  size_t status = 0;
  
  //
  ac.sendGoal(goal);

  const double timeout = 10.0;
  bool finished_before_timeout;
  finished_before_timeout = ac.waitForResult(ros::Duration(timeout));//TODO @tttor: may use a callback fo this

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_DEBUG("Action's terminal state= %s", state.toString().c_str());
      status = 1;
    }
    else {
      ROS_DEBUG("The base failed to move for some reason");
      status = 2;
    }
  }
  else {
    ROS_WARN("TIMEOUT: time is up :(");
    status = 3;
  }

  ROS_INFO_STREAM_COND(debug, "status= " << status);
  ROS_INFO_COND(debug, "send_goal(): END");
  return status;
}

}// namespace rbmt_nav