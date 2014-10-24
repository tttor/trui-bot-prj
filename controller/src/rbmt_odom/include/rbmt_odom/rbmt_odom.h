#ifndef RBMT_ODOM_H_
#define RBMT_ODOM_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

namespace rbmt_odom {

class Odometry {
 public:
  Odometry(ros::NodeHandle nh);
  ~Odometry();

  double x();
  double y();
  double theta();
  ros::Time current_time();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber act_vel_sub_;
  
  //! These are variables for robot pose in reference to the /world 
  double x_, y_, theta_;

  //! Time variables for updating and publishing odom
  ros::Time current_time_, last_time_;

  void act_vel_sub_cb(const geometry_msgs::TwistConstPtr& msg);
  void update(const double& vx, const double& vy, const double& vth);
};

}// namespace rbmt_odom

#endif