#ifndef RBMT_TRACKING_H_
#define RBMT_TRACKING_H_

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace rbmt_tracking {

class Tracker {
 public:
  Tracker(ros::NodeHandle nh);
  ~Tracker();

  void run();
 
 private:
  ros::Publisher cock_pose_pub_;
  ros::NodeHandle nh_;
};

}

#endif