#include <rbmt_sim/rbmt_sim.h>

namespace rbmt_sim {

Simulator::Simulator(ros::NodeHandle nh): nh_(nh) {
  cmd_wheel_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("cmd_wheel_speed", 1, &Simulator::cmd_wheel_speed_sub_cb, this);
  act_wheel_speed_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("act_wheel_speed", 100);
}

Simulator::~Simulator() {

}

void Simulator::cmd_wheel_speed_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg) {
  std::vector<double> cmd_wheel_speed;
  cmd_wheel_speed.resize(3);

  cmd_wheel_speed.at(0) = msg->twist.linear.x;
  cmd_wheel_speed.at(1) = msg->twist.linear.y;
  cmd_wheel_speed.at(2) = msg->twist.linear.z;

  //
  std::vector<double> act_wheel_speed;
  act_wheel_speed = simulate_wheel_speed_realization(cmd_wheel_speed);

  //
  geometry_msgs::TwistStamped act_wheel_speed_msg;
  act_wheel_speed_msg.twist.linear.x = act_wheel_speed.at(0);
  act_wheel_speed_msg.twist.linear.y = act_wheel_speed.at(1);
  act_wheel_speed_msg.twist.linear.z = act_wheel_speed.at(2);

  act_wheel_speed_pub_.publish(act_wheel_speed_msg);
}

std::vector<double> Simulator::simulate_wheel_speed_realization(const std::vector<double>& cmd_wheel_speed) {
  //TODO @tttor: more sophisticated simualation

  return cmd_wheel_speed;
}

}// namespace rbmt_sim