#ifndef RBMT_TRACKING_H_
#define RBMT_TRACKING_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace rbmt_tracking {

typedef void (*voidFuncPtr)(void);

class Tracker {
 public:
  Tracker(ros::NodeHandle nh);
  ~Tracker();

  void run(ros::Rate loop_rate);
  
  void CallBackFunc(int event, int x, int y, int flags, void* userdata);
 
 private:
  ros::Publisher cock_pose_pub_;
  ros::NodeHandle nh_;

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
//=============== variables ========================================//

};

}

#endif