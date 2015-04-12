#ifndef RBMT_TRACKING_H_
#define RBMT_TRACKING_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>

#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <math.h>

namespace rbmt_tracking {

typedef void (*voidFuncPtr)(void);

class Tracker {
 public:
  Tracker(ros::NodeHandle nh);
  ~Tracker();

  void run(ros::Rate loop_rate);

  //! Dummy: publish hardcoded cock end position
  void run_dummy(ros::Rate rate);

  void marker_init();

  void csv_init(const std::string& filepath);
 
 private:
  ros::Publisher end_pose_pub_;
  ros::NodeHandle nh_;
  ros::Publisher end_marker_pub_;

  //=============== VARIABLES ========================================//
  int xVal, yVal, xHover, yHover, fWidth, fLength, rWidth, rLength;
  cv::Point2f tl, tr, bl, br;
  cv::Point2f center, rCenter;
  cv::Mat imgOriginal, imgBuffer, quad, quadCopy;
  std::vector<cv::Point2f> corners;
  double deg, h, l, w, dist;
  visualization_msgs::Marker marker;
//=============== variables ========================================//

};

}

#endif