#ifndef RBMT_BASE_CTRL_H_
#define RBMT_BASE_CTRL_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TwistStamped.h>
#include <rbmt_base_ctrl/rbmt_base_kinematics.h>
#include <Eigen/Dense>

namespace rbmt_base_ctrl {

class BaseCtrl {
 public:
  BaseCtrl(ros::NodeHandle nh);
  ~BaseCtrl();

 private:
  ros::NodeHandle nh_;  
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber act_wheel_speed_sub_;
  ros::Publisher cmd_wheel_speed_pub_;
  ros::Publisher act_vel_pub_;
  
  /*!
   * \brief Input speed command vector represents the desired speed requested by the node. 
   * Note that this may differ from the current commanded speed due to acceleration limits imposed by the controller.
   */
  geometry_msgs::Twist cmd_vel_;

  /*!
   * \brief ...
   */
  geometry_msgs::Twist act_vel_;

  /*!
   * \brief speed command vector used internally to represent the current commanded speed
   */
  geometry_msgs::Twist raw_cmd_vel_;

  /*!
   * \brief maximum translational velocity magnitude allowable
   */
  double max_translational_velocity_;

  /*!
   * \brief maximum rotational velocity magnitude allowable
   */
  double max_rotational_velocity_;

  /*! 
   * \brief class where the robot's information is computed and stored
   * @return BaseKinematic instance that is being used by this controller
   */
  BaseKinematics base_kinematics_;

  /*!
   * \brief a srv client for sending wheel speeds to the onboard controllers
  */
  ros::ServiceClient rbmt_proxy_srv_client_;

  /*!
   * \brief a routine of a subscriber to the topic of cmd_vel
   */
  void cmd_vel_sub_cb(const geometry_msgs::TwistConstPtr& msg);

  /*!
   * \brief a routine of a subscriber to the topic of act_wheel_speed
   */
  void act_wheel_speed_sub_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  /*! 
  * \brief fix the raw cmd_vel to adhere some constraint, e.g. linear max vel, angular max vel
  * @param cmd_vel Velocity command of the base in m/s and rad/s
  */
  void fix_cmd_vel(const geometry_msgs::Twist &cmd_vel);

  /*!
   * \brief computes the desired wheel speeds given the desired base speed
   */
  void compute_wheel_speed();

  /*!
  * \brief sends the desired wheel speeds to the wheel controllers, i.e the onboard arduinos.
  * In this project, this publish geometry_msgs::TwistStamped msg to /rbmt_serial/setpoint/cmd_vel
  * \return whether ...
  */
  bool send_wheel_speed();

  /*!
  * \brief ...
  */
  void set_act_wheel_speed(const geometry_msgs::TwistStampedConstPtr& msg);

  /*!
  * \brief ...
  */
  void compute_act_vel();
};

}// namespace rbmt_base_ctrl 
#endif