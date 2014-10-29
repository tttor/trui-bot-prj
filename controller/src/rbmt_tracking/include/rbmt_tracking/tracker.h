#ifndef RBMT_TRACKING_H_
#define RBMT_TRACKING_H_

#include <geometry_msgs/Pose.h>

namespace rbmt_tracking {

class Tracker {
 public:
  Tracker();
  ~Tracker();

  void run();
 
 private:
  ros::Publisher cock_pose_pub_;
};

}

#endif