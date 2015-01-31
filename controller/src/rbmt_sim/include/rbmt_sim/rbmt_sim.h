#ifndef RBMT_SIM_H_
#define RBMT_SIM_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <iostream>

namespace rbmt_sim {

class Simulator {
 public:
  Simulator(ros::NodeHandle nh);
  ~Simulator();
  void run(ros::Rate rate);

 private:
  ros::NodeHandle nh_;

  ros::Subscriber cmd_wheel_speed_sub_;
  ros::Subscriber cmd_service_sub_;

  ros::Publisher act_wheel_speed_pub_;
  ros::Publisher act_joint_state_pub_;

  sensor_msgs::JointState act_joint_state_;
  
  void cmd_wheel_speed_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);
  void cmd_service_sub_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  std::vector<double> simulate_wheel_speed_realization(const std::vector<double>& cmd_wheel_speed);
};

}// namespace rbmt_sim

#endif