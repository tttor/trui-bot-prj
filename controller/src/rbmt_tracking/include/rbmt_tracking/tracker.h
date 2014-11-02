#ifndef RBMT_TRACKING_H_
#define RBMT_TRACKING_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>

namespace rbmt_tracking {

typedef void (*voidFuncPtr)(void);

class Tracker {
 public:
  Tracker(ros::NodeHandle nh);
  ~Tracker();

  void run(ros::Rate loop_rate);

  //! Dummy: publish hardcoded cock end position
  void run_dummy(ros::Rate rate);
  
  void CallBackFunc(int event, int x, int y, int flags, void* userdata);

  void marker_init();
 
 private:
  ros::Publisher cock_pose_pub_;
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;

  //=============== VARIABLES ========================================//
  int xVal;
  int yVal;
  int xHover;
  int yHover;
  cv::Point2f tl;
  cv::Point2f tr;
  cv::Point2f bl;
  cv::Point2f br;
  cv::Point2f center;
  cv::Mat imgOriginal;
  cv::Mat imgBuffer;
  cv::Mat quad;
  std::vector<cv::Point2f> corners;
  visualization_msgs::Marker marker;
//=============== variables ========================================//

};

}

#endif