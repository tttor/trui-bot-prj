#include <rbmt_sim/rbmt_sim.h>

namespace rbmt_sim {

Simulator::Simulator(ros::NodeHandle nh): nh_(nh) {
  cmd_wheel_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("cmd_wheel_speed", 1, &Simulator::cmd_wheel_speed_sub_cb, this);
  cmd_service_sub_ = nh_.subscribe<>("cmd_service", 1, &Simulator::cmd_service_sub_cb, this);

  act_wheel_speed_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("act_wheel_speed", 100);
  act_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("act_joint_state",100);

  //
  const size_t n_joint = 1;// #active joint

  act_joint_state_.name.resize(n_joint);
  act_joint_state_.position.resize(n_joint);
  act_joint_state_.velocity.resize(n_joint);
  act_joint_state_.effort.resize(n_joint);

  act_joint_state_.name.at(0) = "roll_joint";
  act_joint_state_.position.at(0) = 0.0;
  act_joint_state_.velocity.at(0) = 0.0;
  act_joint_state_.effort.at(0) = 0.0;

  ROS_INFO("Simulator: up and running");
}

Simulator::~Simulator() {

}

void Simulator::cmd_service_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg) {
  //TODO @tttor: may define service_joint_state
  const double service_joint_state = M_PI;
  const double yaw = tf::getYaw(msg->pose.orientation);
  
  if (yaw==service_joint_state) {
    act_joint_state_.position.at(0) = service_joint_state;
  } 
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
  const float realization_factor = 1.0;
  geometry_msgs::TwistStamped act_wheel_speed_msg;
  act_wheel_speed_msg.twist.linear.x = act_wheel_speed.at(0) * realization_factor;
  act_wheel_speed_msg.twist.linear.y = act_wheel_speed.at(1) * realization_factor;
  act_wheel_speed_msg.twist.linear.z = act_wheel_speed.at(2) * realization_factor;

  act_wheel_speed_pub_.publish(act_wheel_speed_msg);
}

std::vector<double> Simulator::simulate_wheel_speed_realization(const std::vector<double>& cmd_wheel_speed) {
  //TODO @tttor: more sophisticated simualation

  return cmd_wheel_speed;
}

void Simulator::run(ros::Rate rate) {
  while (ros::ok()) {
    //TODO @tttor: after service, return the state of joint pose

    act_joint_state_pub_.publish(act_joint_state_);

    ros::spinOnce();
    rate.sleep();
  }

}

}// namespace rbmt_sim