#ifndef RBMT_SIM_H_
#define RBMT_SIM_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector>

namespace rbmt_sim {

class Simulator {
 public:
  Simulator(ros::NodeHandle nh);
  ~Simulator();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_wheel_speed_sub_;
  ros::Publisher act_wheel_speed_pub_;
  
  void cmd_wheel_speed_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);
  std::vector<double> simulate_wheel_speed_realization(const std::vector<double>& cmd_wheel_speed);
};

}// namespace rbmt_sim

#endif