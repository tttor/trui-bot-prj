#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <iostream>
#include <log4cxx/logger.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

size_t send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac) {
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

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "nav_ctrl");
  ros::NodeHandle nh;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  //
  const string action_server_name = "move_base";
  MoveBaseClient ac(action_server_name);
  ac.waitForServer();

  //
  vector<move_base_msgs::MoveBaseGoal> goals;
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";// the reference frame of the goal pose
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  goals.push_back(goal);

  goal.target_pose.header.frame_id = "map";// the reference frame of the goal pose
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  goals.push_back(goal);

  goal.target_pose.header.frame_id = "map";// the reference frame of the goal pose
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 3.5;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  goals.push_back(goal);

  //
  for (size_t i=0; i<goals.size(); ++i) {
    send_goal(goals.at(i), ac);  
    ros::Duration(1.0).sleep();
  }
  
  return 0;
}