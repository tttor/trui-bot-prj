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
  // ROS_DEBUG("Odometry::act_vel_sub_cb(): BEGIN");
  
  current_time_ = ros::Time::now();
  update(msg->linear.x, msg->linear.y, msg->angular.z);
  last_time_ = current_time_;

  // ROS_DEBUG("Odometry::act_vel_sub_cb(): END");
}

void Odometry::update(const double& vx, const double& vy, const double& vth) {
  const bool debug = !false;

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time_ - last_time_).toSec();// TODO @tttor: fix me!
  dt = 1.0;// TODO @tttor: remove me!
  
  // All velocity values refer to the vel of /base_link to a fixed global frame, i.e. /map, /world
  ROS_DEBUG_STREAM_COND(debug, "vx= " << vx << "; vy= " << vy << "; vth= " << vth << "; dt= " << dt);
  
  // All deltas are computed w.r.t a fixed global frame, i.e. /map, /world
  double dx, dy, dth;
  dx = vx * dt;
  dy = vy * dt;
  dth = vth * dt;

  ROS_DEBUG_STREAM_COND(debug, "dx= " << dx << "; dy= " << dy << "; dth= " << dth);

  //
  x_ += dx;
  y_ += dy;
  theta_ += dth;   

  ROS_DEBUG_STREAM_COND(debug, "x_= " << x_ << "; y_= " << y_ << "; theta_= " << theta_);
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