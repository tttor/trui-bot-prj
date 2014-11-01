#ifndef RBMT_TELEOP_H_
#define RBMT_TELEOP_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <map>
#include <cmath>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>


namespace rbmt_teleop {

class TeleopTranslator {
 public:
  TeleopTranslator(ros::NodeHandle nh);
  ~TeleopTranslator();
  void run();
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher cmd_service_pub_;

  size_t n_axes_;
  size_t n_button_;

  std::vector<float> axes_;
  std::vector<int32_t> buttons_;

  std::vector<float> axis_mins_;
  std::vector<float> axis_maxs_;
  std::vector<float> axis_normals_;

  std::map<std::string, float> vel_param_;

  //! Map the button or axis names to their number in either buttons_ or axes_
  std::map<std::string, size_t> num_;

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

  size_t send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac);
};

}// namespace rbmt_teleop

#endif