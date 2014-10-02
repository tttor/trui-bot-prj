#include <rbmt_odom/rbmt_odom.h>

namespace rbmt_odom {

Odometry::Odometry(ros::NodeHandle nh): nh_(nh) {
  // Init the odometry-base robot pose 
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;

  current_time_ = ros::Time::now();
  last_time_ = current_time_;

  act_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("act_vel", 1, &Odometry::act_vel_sub_cb, this);
}

Odometry::~Odometry() {
  
}

void Odometry::act_vel_sub_cb(const geometry_msgs::TwistConstPtr& msg) {
  current_time_ = ros::Time::now();
  update(msg->linear.x, msg->linear.y, msg->angular.z);
  last_time_ = current_time_;
}

void Odometry::update(const double& vx, const double& vy, const double& vth) {
  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time_ - last_time_).toSec();
  
  double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * dt;
  double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * dt;
  double delta_th = vth * dt;

  //
  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_th;
}

double Odometry::x() {
  return x_;
}

double Odometry::y() {
  return y_;
}

double Odometry::theta() {
  return theta_;
}

ros::Time Odometry::current_time() {
  return current_time_;
}

}// namespace rbmt_odom