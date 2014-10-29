#include <rbmt_tracking/tracker.h>

namespace rbmt_tracking{

Tracker::Tracker(ros::NodeHandle nh): nh_(nh) {
  cock_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("cock_pose", 1);
}
Tracker::~Tracker() {
  
}

void Tracker::run() {
  ros::Rate loop_rate(10);
  
  while (ros::ok()) {
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
  }
} 

}