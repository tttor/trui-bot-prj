#include <rbmt_move_base/move_base.h>
#include <geometry_msgs/Twist.h>

namespace rbmt_move_base {

MoveBase::MoveBase(tf::TransformListener& tf_listener): tf_listener_(tf_listener), as_(NULL) {
  using namespace std;

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  
  global_planner_ = new GlobalPlanner();
  local_planner_ = new LocalPlanner();

  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
	as_->start();
  ROS_INFO("move_base: up and running");
}

MoveBase::~MoveBase() { 
  delete global_planner_;
  delete local_planner_;
  if(as_ != NULL) delete as_;
}

geometry_msgs::PoseStamped MoveBase::get_pose() {
  tf::StampedTransform transform;

  try {
    tf_listener_.lookupTransform("/link_base", "/world", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::PoseStamped pose;

  pose.header.frame_id = "/world";
  pose.pose.position.x = transform.getOrigin().x();
  pose.pose.position.y = transform.getOrigin().y();
  pose.pose.position.z = transform.getOrigin().z();
  pose.pose.orientation.x = transform.getRotation().x();
  pose.pose.orientation.y = transform.getRotation().y();
  pose.pose.orientation.z = transform.getRotation().z();
  pose.pose.orientation.w = transform.getRotation().w();

  return pose;
}

void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal) {
  using namespace std;
  ROS_INFO("move_base: executeCb: BEGIN");

  // TODO @tttor: do planning, then publish velocity to the cmd_vel topic
  geometry_msgs::PoseStamped start_pose, goal_pose;
  start_pose = get_pose();
  goal_pose.pose = move_base_goal->target_pose.pose;

  global_planner_->plan(start_pose, goal_pose, &global_plan_);

  //
  std::vector<geometry_msgs::Twist> local_plan;
  local_planner_->plan(global_plan_, &local_plan);

  // Publish the local plan 
  for (size_t i=0; i<local_plan.size(); ++i) {
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;// as yaw rate
    
    vel_pub_.publish(cmd_vel);  
  }

  // // From goal_functions.cpp
  // //check to see if we've reached the goal position
  // if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
  //   //check to see if the goal orientation has been reached
  //   if(fabs(getGoalOrientationAngleDifference(global_pose, goal_th)) <= yaw_goal_tolerance) {
  //     //make sure that we're actually stopped before returning success
  //     if(stopped(base_odom, rot_stopped_vel, trans_stopped_vel))
  //       return true;
  //   }
  // }

  as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");  
  ROS_INFO("move_base: executeCb: END");
}

}// namespace rbmt_move_base