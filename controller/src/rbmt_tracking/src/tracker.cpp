#include <rbmt_tracking/tracker.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace rbmt_tracking{

Tracker::Tracker(ros::NodeHandle nh): nh_(nh) {
  cock_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("cock_pose", 1);
}
Tracker::~Tracker() {
  
}

void Tracker::run() {
  
  using namespace std;
  using namespace cv;
  
  ros::Rate loop_rate(10);
  
  VideoCapture cap(1); //capture the video from webcam
  namedWindow("Camera", 1);

  while (ros::ok()) {
    
    Mat imgOriginal;
    cap.read(imgOriginal);
    imshow("Camera", imgOriginal);
    
    geometry_msgs::Pose pose;

    pose.position.x = 0.0;
    pose.position.y = 1.0;
    pose.position.z = 2.0;

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    cock_pose_pub_.publish(pose);
    ros::spinOnce();

    loop_rate.sleep();
    if (waitKey(1) == 27) //wait for 'esc' key press for 1ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed" << endl;
      break;
    }
  }
} 

}
