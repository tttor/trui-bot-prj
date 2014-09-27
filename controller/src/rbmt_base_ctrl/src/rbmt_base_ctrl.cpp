#include <rbmt_base_ctrl/rbmt_base_ctrl.h>

namespace rbmt_base_ctrl {

BaseCtrl::BaseCtrl(ros::NodeHandle nh): nh_(nh) {
  raw_cmd_vel_.linear.x = 0; raw_cmd_vel_.linear.y = 0; raw_cmd_vel_.angular.z = 0;
  cmd_vel_.linear.x = 0; cmd_vel_.linear.y = 0; cmd_vel_.angular.z = 0;

  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &BaseCtrl::cmd_vel_sub_cb, this);

  rbmt_proxy_srv_client_ = nh_.serviceClient<rbmt_proxy::SendWheelSpeed>("send_wheel_speed");

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
  rbmt_proxy::SendWheelSpeed srv;
  srv.request.wheel_speeds.resize(base_kinematics_.wheels.size());
  
  for (size_t i = 0; i<base_kinematics_.wheels.size(); ++i) {
    srv.request.wheel_speeds.at(i) = base_kinematics_.wheels.at(i).wheel_speed_cmd; 
  }
  
  if (rbmt_proxy_srv_client_.call(srv)) {
    ROS_INFO_STREAM("response.msg= " << srv.response.msg);
  }
  else {
    ROS_ERROR("Failed to call service SendWheelSpeed of rbmt_proxy");
    return false;
  }

  return true;
}

}// namespace rbmt_base_ctrl
