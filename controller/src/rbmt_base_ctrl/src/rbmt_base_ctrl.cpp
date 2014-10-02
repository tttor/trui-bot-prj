#include <rbmt_base_ctrl/rbmt_base_ctrl.h>

namespace rbmt_base_ctrl {

BaseCtrl::BaseCtrl(ros::NodeHandle nh): nh_(nh) {
  raw_cmd_vel_.linear.x = 0; raw_cmd_vel_.linear.y = 0; raw_cmd_vel_.angular.z = 0;
  cmd_vel_.linear.x = 0; cmd_vel_.linear.y = 0; cmd_vel_.angular.z = 0;

  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &BaseCtrl::cmd_vel_sub_cb, this);
  act_wheel_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("act_wheel_speed", 1, &BaseCtrl::act_wheel_speed_sub_cb, this);
  
  act_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("act_vel", 100);  
  cmd_wheel_speed_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_wheel_speed", 100);

  //Get params from param server
  nh_.param<double> ("max_translational_velocity", max_translational_velocity_,0.5);
  nh_.param<double> ("max_rotational_velocity", max_rotational_velocity_, 10.0);
  // nh_.param<double> ("timeout", timeout_, 1.0);

  base_kinematics_.init();
  // new_cmd_available_ = false; 
  // last_publish_time_ = ros::Time(0.0);
}

BaseCtrl::~BaseCtrl() {
  cmd_vel_sub_.shutdown();
}

void BaseCtrl::act_wheel_speed_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg) {
  set_act_wheel_speed(msg);
  compute_act_vel();
  act_vel_pub_.publish(act_vel_);
}

void BaseCtrl::set_act_wheel_speed(const geometry_msgs::TwistStampedConstPtr& msg) {
  base_kinematics_.wheels.at(0).wheel_speed_actual = msg->twist.linear.x;
  base_kinematics_.wheels.at(1).wheel_speed_actual = msg->twist.linear.y;
  base_kinematics_.wheels.at(2).wheel_speed_actual = msg->twist.linear.z;
}

void BaseCtrl::compute_act_vel() {
  //TODO: do the math, the inverse of compute_wheel_speed given cmd_vel
  // do something with wheel.wheel_speed_actual and act_vel_
}

void BaseCtrl::cmd_vel_sub_cb(const geometry_msgs::TwistConstPtr& msg) {
  fix_cmd_vel(*msg);
  compute_wheel_speed();
  send_wheel_speed();
}

void BaseCtrl::fix_cmd_vel(const geometry_msgs::Twist& raw_cmd_vel) {
  raw_cmd_vel_ = raw_cmd_vel;
  cmd_vel_ = raw_cmd_vel_;

  // TODO @tttor: do the real job
  // double vel_mag = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
  // double clamped_vel_mag = filters::clamp(vel_mag,-max_translational_velocity_, max_translational_velocity_);
  // if(vel_mag > EPS)
  // {
  //   raw_cmd_vel.linear.x = raw_cmd_vel.linear.x * clamped_vel_mag / vel_mag;
  //   raw_cmd_vel.linear.y = raw_cmd_vel.linear.y * clamped_vel_mag / vel_mag;
  // }
  // else
  // {
  //   raw_cmd_vel.linear.x = 0.0;
  //   raw_cmd_vel.linear.y = 0.0;
  // }
  // raw_cmd_vel.angular.z = filters::clamp(cmd_vel.angular.z, -max_rotational_velocity_, max_rotational_velocity_);

  ROS_DEBUG("BaseController:: command received: %f %f %f",raw_cmd_vel.linear.x,raw_cmd_vel.linear.y,raw_cmd_vel.angular.z);
  ROS_DEBUG("BaseController:: command current: %f %f %f", cmd_vel_.linear.x,cmd_vel_.linear.y,cmd_vel_.angular.z);
  // ROS_DEBUG("BaseController:: clamped vel: %f", clamped_vel_mag);
  // ROS_DEBUG("BaseController:: vel: %f", vel_mag);
}

void BaseCtrl::compute_wheel_speed() {
  // Do the math here
  // ...
  double dummy_speed = 0.0;

  // Set
  for (size_t i=0; i<base_kinematics_.wheels.size(); ++i) {
    base_kinematics_.wheels.at(i).wheel_speed_cmd = dummy_speed;
  }
}

bool BaseCtrl::send_wheel_speed() {
  geometry_msgs::TwistStamped msg;

  msg.twist.linear.x = base_kinematics_.wheels.at(1).wheel_speed_cmd;
  msg.twist.linear.y = base_kinematics_.wheels.at(2).wheel_speed_cmd;
  msg.twist.linear.z = base_kinematics_.wheels.at(3).wheel_speed_cmd;

  cmd_wheel_speed_pub_.publish(msg);
}

}// namespace rbmt_base_ctrl
