#ifndef RBMT_MOVE_BASE_ACTION_H_
#define RBMT_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>
#include <iostream>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace rbmt_move_base {

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBase {
 public:
 	MoveBase();
 	~MoveBase();

 private:
 	void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

 	MoveBaseActionServer* as_;
};

}// namespace rbmt_move_base
#endif 