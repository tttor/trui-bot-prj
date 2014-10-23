#include <rbmt_joy/rbmt_joy_translator.h>

namespace rbmt_joy {

JoyTranslator::JoyTranslator(ros::NodeHandle nh): nh_(nh) {
  //
  n_axes_ = 8;
  n_button_ = 11;

  axes_.resize(n_axes_);
  axis_mins_.resize(n_axes_);
  axis_maxs_.resize(n_axes_);
  axis_normals_.resize(n_axes_);
  buttons_.resize(n_button_);

  //
  axis_mins_.at(0) = -1.0;
  axis_maxs_.at(0) =  1.0;
  axis_normals_.at(0) = 0.0;

  axis_mins_.at(1) = -1.0;
  axis_maxs_.at(1) =  1.0;
  axis_normals_.at(1) = 0.0;

  axis_mins_.at(5) = -1.0;
  axis_maxs_.at(5) =  1.0;
  axis_normals_.at(5) = 1.0;

  vel_param_["x_vel_min"] = -1.0;
  vel_param_["x_vel_max"] =  1.0;
  vel_param_["y_vel_min"] = -1.0;
  vel_param_["y_vel_max"] =  1.0;
  vel_param_["theta_vel_min"] = 0.0;
  vel_param_["theta_vel_max"] = M_PI;

  //
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &JoyTranslator::joy_sub_cb, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
}

JoyTranslator::~JoyTranslator() {

}

void JoyTranslator::joy_sub_cb(const sensor_msgs::JoyConstPtr& msg) {
  axes_ = msg->axes;
  buttons_ = msg->buttons;
}

void JoyTranslator::run() {
  // axes(5): RT _must_ be initialized because _before_ first update _only_, axes_.at(5) has a normal value of 0, plus, if another axes is pushed before RT is initialized, then RT changes to not-normal value; weird!
  bool RT_initialized = false;

  while (ros::ok() and !RT_initialized) {
    ROS_INFO("Waiting for RT (axis(5)) to be initialized.");
    if (axes_.at(5)==axis_normals_.at(5)) break;
    ros::spinOnce();
  }
  ROS_INFO("RT (axis(5)) is initialized.");

  ros::Rate rate(10);
  while (ros::ok()) {
    // Set the values based on joy readings
    double x_vel, y_vel, theta_vel;
    
    x_vel = (reverse(axes_.at(0)) / (axis_range_ratio(0)*axis_range(0))) * (axis_range_ratio(0)*vel_range("x_vel"));
    y_vel = (axes_.at(1) / (axis_range_ratio(1)*axis_range(1))) * (axis_range_ratio(1)*vel_range("y_vel"));
    theta_vel = (std::abs(axes_.at(5)-1.0) / (axis_range_ratio(5)*axis_range(5))) * (axis_range_ratio(5)*vel_range("theta_vel"));

    // Publish
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = x_vel;
    cmd_vel.linear.y = y_vel;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = theta_vel;

    // ROS_DEBUG_STREAM("cmd_vel.linear.x= "<< cmd_vel.linear.x);
    // ROS_DEBUG_STREAM("cmd_vel.linear.y= "<< cmd_vel.linear.y);
    // ROS_DEBUG_STREAM("cmd_vel.angular.z= "<< cmd_vel.angular.z);

    cmd_vel_pub_.publish(cmd_vel);  

    //
    ros::spinOnce();
    rate.sleep();
  }
}

float JoyTranslator::axis_range(const size_t& ith) {
  return axis_maxs_.at(ith) - axis_mins_.at(ith);
}

float JoyTranslator::vel_range(const std::string type) {
  return vel_param_[std::string(type+"_max")] - vel_param_[std::string(type+"_min")]; 
}

float JoyTranslator::reverse(const float& val) {
  return -1.0 * val;
}

float JoyTranslator::axis_range_ratio(const size_t& ith) {
  return std::abs(axis_normals_.at(ith)-axis_mins_.at(ith)) / axis_range(ith);
}

}// namespace rbmt_joy
