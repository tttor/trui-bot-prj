#ifndef RBMT_JOY_TRANSLATOR_H_
#define RBMT_JOY_TRANSLATOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace rbmt_joy {

class JoyTranslator {
 public:
  JoyTranslator(ros::NodeHandle nh_);
  ~JoyTranslator();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_vel_pub_;

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

};
}// namespace rbmt_joy

#endif

