#include <rbmt_odom/rbmt_odom.h>

int main(int argc, char** argv) {
  using namespace std;

  ros::init(argc, argv, "odom");
  ros::NodeHandle nh;

  rbmt_odom::Odometry odom(nh);
  tf::TransformBroadcaster odom_broadcaster;// this _must_ be outside the loop

  // TODO: Publish the odometry message (nav_msgs::Odometry) over ROS, e.g., for the navigation stack
  ros::Rate rate(10.0);
  while (ros::ok()) {
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = ros::Time::now();//do _not_ use odom.current_time();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "odom";

    odom_trans.transform.translation.x = odom.x();
    odom_trans.transform.translation.y = odom.y();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom.theta());

    odom_broadcaster.sendTransform(odom_trans);

    rate.sleep();
  }

  return(0);
}