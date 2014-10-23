#ifndef RBMT_JOY_TRANSLATOR_H_
#define RBMT_JOY_TRANSLATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <map>
#include <cmath>

namespace rbmt_joy {

class JoyTranslator {
 public:
  JoyTranslator(ros::NodeHandle nh_);
  ~JoyTranslator();
  void run();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_vel_pub_;

  size_t n_axes_;
  size_t n_button_;

  std::vector<float> axes_;
  std::vector<int32_t> buttons_;

  std::vector<float> axis_mins_;
  std::vector<float> axis_maxs_;
  std::vector<float> axis_normals_;

  std::map<std::string, float> vel_param_;

/*!
 * axes.at(0) horizontal left analog
 * axes.at(1) vertical left analog
 * axes.at(2) LT
 * axes.at(3) horizontal right analog
 * axes.at(4) vertical right analog
 * axes.at(5) RT 
 * axes.at(6) horizontal left buttons
 * axes.at(7) vertical left buttons
 *
 * buttons.at(0) button A
 * buttons.at(1) button B
 * buttons.at(2) button X
 * buttons.at(3) button Y
 * buttons.at(4) LB
 * buttons.at(5) RB
 * buttons.at(6) button BACK
 * buttons.at(7) button START
 * buttons.at(8) 
 * buttons.at(9) left analog click
 * buttons.at(10) right analog click
 */
  void joy_sub_cb(const sensor_msgs::JoyConstPtr& msg);

  float axis_range(const size_t& ith);

  float axis_range_ratio(const size_t& ith);

  float vel_range(const std::string type);

  float reverse(const float& val);
};

}// namespace rbmt_joy

#endif