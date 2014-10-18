#include <rbmt_joy/rbmt_joy_translator.h>

namespace rbmt_joy {

JoyTranslator::JoyTranslator(ros::NodeHandle nh): nh_(nh) {
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &JoyTranslator::joy_sub_cb, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
}

JoyTranslator::~JoyTranslator() {

}

void JoyTranslator::joy_sub_cb(const sensor_msgs::JoyConstPtr& msg) {
  //
  double ax0, ax1, ax2, ax3, ax4, ax5, ax6, ax7;
  int but0, but1, but2, but3, but4, but5, but6, but7, but8, but9, but10;
  ax0 = msg->axes.at(0);
  ax1 = msg->axes.at(1);
  ax2 = msg->axes.at(2);
  ax3 = msg->axes.at(3);
  ax4 = msg->axes.at(4);
  ax5 = msg->axes.at(5);
  ax6 = msg->axes.at(6);
  ax7 = msg->axes.at(7);

  but0 = msg->buttons.at(0);
  but1 = msg->buttons.at(1);
  but2 = msg->buttons.at(2);
  but3 = msg->buttons.at(3);
  but4 = msg->buttons.at(4);
  but5 = msg->buttons.at(5);
  but6 = msg->buttons.at(6);
  but7 = msg->buttons.at(7);
  but8 = msg->buttons.at(8);
  but9 = msg->buttons.at(9);
  but10 = msg->buttons.at(10);

  // ROS_INFO_STREAM("ax0: " << ax0);
  // ROS_INFO_STREAM("ax1: " << ax1);
  
  // translate
  /*! 
   * axes.at(0) ranges from -1(right) to 1(left)
   * axes.at(1) ranges from -1(down) to 1(up)
   * 
   * theoritically maximum velocity of motor
   * omega = 500 rotation/minute
   * wheel radius = 0.2 meter
   * maximum velocity (m/s) = omega * wheel circumference / 60 seconds
   *                        = 500 * 2 * pi * 0.2 / 60
   *                        = 10.47 m/s 
   * 
   * assume: maximum linear x-axis velocity = 9 m/s
   *         maximum linear y-axis velocity = 9 m/s
   */

  const double max_x_vel = 9;
  const double max_y_vel = 9;

  double x_vel, y_vel;

  x_vel = (-1) * ax0 * max_x_vel;
  y_vel = ax1 * max_y_vel;

  ROS_INFO_STREAM("x_vel = " << x_vel);
  ROS_INFO_STREAM("y_vel = " << y_vel);
  
  // publish
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x = x_vel;// hasil translate
  cmd_vel.linear.y = y_vel;

  cmd_vel_pub_.publish(cmd_vel);
}

}// namespace rbmt_joy
