#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
  using namespace std;

  ros::init(argc, argv, "odom_loc");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener(ros::Duration(10));
  tf::TransformBroadcaster pose_broadcaster;// this _must_ be outside the loop

  //TODO may consider subcribing to odometry msg if it is available
  ROS_INFO("waitForTransform /odom w.r.t /map");
  tf_listener.waitForTransform("map", "odom", ros::Time::now(), ros::Duration(5.0));

  ros::Rate rate(100);
  while ( ros::ok() ) {
    // Look up the transformation of /odom in reference to /world
    tf::StampedTransform odom_tf;

    try {
      tf_listener.lookupTransform("map", "odom", ros::Time(0), odom_tf);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // ROS_INFO_STREAM("odom_tf.getOrigin().x()= " << odom_tf.getOrigin().x());
    
    // Broadcast the transformation
    geometry_msgs::TransformStamped pose_tf;

    pose_tf.header.stamp = ros::Time::now();
    pose_tf.header.frame_id = "map";
    pose_tf.child_frame_id = "base_link";
    pose_tf.transform.translation.x = odom_tf.getOrigin().x();
    pose_tf.transform.translation.y = odom_tf.getOrigin().y();
    pose_tf.transform.translation.z = odom_tf.getOrigin().z(); 
    pose_tf.transform.rotation.x = odom_tf.getRotation().x();
    pose_tf.transform.rotation.y = odom_tf.getRotation().y();
    pose_tf.transform.rotation.z = odom_tf.getRotation().z();
    pose_tf.transform.rotation.w = odom_tf.getRotation().w();

    // ROS_INFO_STREAM("pose_tf.transform.translation.x= " << pose_tf.transform.translation.x);
    pose_broadcaster.sendTransform(pose_tf);

    rate.sleep();
  }

  return(0);
}