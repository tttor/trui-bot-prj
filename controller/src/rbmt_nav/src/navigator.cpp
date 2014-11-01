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
  goal.target_pose.header.frame_id = "map";// the reference frame of the goal pose
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = msg->pose.position.x;
  goal.target_pose.pose.position.y = msg->pose.position.y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  send_goal(goal, ac);
}



size_t Navigator::send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac) {
  ROS_INFO("send_goal(): BEGIN");
  size_t status = 0;
  
  //
  ac.sendGoal(goal);

  bool finished_before_timeout;
  finished_before_timeout = ac.waitForResult(ros::Duration(5.0));//TODO @tttor: may use a callback fo this

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_DEBUG("Action's terminal state= %s", state.toString().c_str());
      status = 1;
    }
    else {
      ROS_DEBUG("The base failed to move forward 2 meters for some reason");
      status = 2;
    }
  }
  else {
    ROS_WARN("TIMEOUT: time is up :(");
    status = 3;
  }

  ROS_INFO("send_goal(): BEGIN");
  return status;
}

}// namespace rbmt_nav