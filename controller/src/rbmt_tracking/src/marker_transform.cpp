
#include <ros/ros.h>
#include <ros/console.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/passthrough.h>

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

#include <sstream>
#include <vector>

ros::Publisher transform_pub;
ros::Publisher marker_pub;

visualization_msgs::Marker marker, line, points;

void
marker_init() {
 // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
	uint32_t line_strip = visualization_msgs::Marker::LINE_STRIP;
	uint32_t points_mark =  visualization_msgs::Marker::POINTS;

  points.header.frame_id = line.header.frame_id = marker.header.frame_id = "world";
  points.header.stamp = line.header.stamp = marker.header.stamp = ros::Time::now();
  marker.type = shape;
  line.type = line_strip;
  points.type = points_mark;
  points.action = line.action = marker.action = visualization_msgs::Marker::ADD;

  line.id = 0;
  points.id = 1;
  marker.id = 2;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  points.scale.x = 0.1;
  points.scale.y = 0.1;

  points.color.r = 0.0f;
  points.color.g = 0.0f;
  points.color.b = 1.0f;
  points.color.a = 1.0;

  line.scale.x = 0.05;	
  line.color.r = 1.0f;
  line.color.g = 0.0f;
  line.color.b = 0.0f;
  line.color.a = 1.0;
}

void 
transformer (const geometry_msgs::PoseStamped& sPose)
{

  marker.lifetime = ros::Duration();

  transform_pub.publish(sPose);
  marker.pose = sPose.pose;

  geometry_msgs::Point p;
  p.x = sPose.pose.position.x;
  p.y = sPose.pose.position.y; 
  p.z = sPose.pose.position.z;

  line.lifetime = ros::Duration(5);
  points.lifetime = ros::Duration(5);

  line.pose.orientation.w = 1.0;
  points.pose.orientation.w = 1.0;
  line.points.push_back(p);
  points.points.push_back(p);

  marker_pub.publish(marker);
  marker_pub.publish(line);
  marker_pub.publish(points);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "marker_transform");
  ros::NodeHandle nh;

  // Initialize marker
  marker_init();

  // Create a ROS subscriber for raw cock pose
  ros::Subscriber sub = nh.subscribe ("cock_pose", 1, transformer);
  
  transform_pub = nh.advertise<geometry_msgs::PoseStamped>("transformed_pose", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Spin
  ros::spin();
}
