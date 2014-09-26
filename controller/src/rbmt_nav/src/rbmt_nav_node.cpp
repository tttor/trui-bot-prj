#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "nav_ctrl");
  ros::NodeHandle nh;

  const string action_server_name = "move_base";
  MoveBaseClient ac(action_server_name);
  ac.waitForServer();

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 2 meters forward
  goal.target_pose.header.frame_id = "base_link";// where do we specify the reference frame?
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  bool finished_before_timeout;
  finished_before_timeout = ac.waitForResult(ros::Duration(5.0));//TODO @tttor: may use a callback fo this

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac.getState();

    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Action's terminal state= %s", state.toString().c_str());
      ROS_INFO("Hooray, the base moved 2 meters forward");
    }
    else {
      ROS_INFO("The base failed to move forward 2 meters for some reason");
    }
  }

  return 0;
}