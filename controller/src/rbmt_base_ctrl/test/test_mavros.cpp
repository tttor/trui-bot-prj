#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "mavros_tester");
  ros::NodeHandle nh;

  ros::Publisher pub;
  pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint/cmd_vel", 100);

  ros::Rate rate(10);
  while (ros::ok()) {

    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = 1.11;
    msg.twist.linear.y = 2.22;
    msg.twist.linear.z = 3.33;
    msg.twist.angular.z = 4.44;

    pub.publish(msg);
    ros::spinOnce();

    rate.sleep();
  }  

  return(0);
}