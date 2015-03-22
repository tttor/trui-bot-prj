
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
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

#include <sstream>
#include <vector>
#include <math.h>

#define _USE_MATH_DEFINES

ros::Publisher transform_pub;
ros::Publisher marker_pub;
ros::Publisher move_pub;

geometry_msgs::Point last, p;
geometry_msgs::PoseStamped black_pose, white_pose, final_pose;
geometry_msgs::Twist move;

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


  marker.lifetime = ros::Duration(0.001);
}

void 
transformer_white (const geometry_msgs::PoseStamped& sPose)
{

  float r, th;
  float x_temp = sPose.pose.position.x;
  float y_temp = sPose.pose.position.y;
  float z_temp = sPose.pose.position.z;

  // angular transform on x - y plane
  r = sqrt((z_temp*z_temp) + (x_temp*x_temp));
  th = atan(z_temp/x_temp);
  th = th + (90*M_PI/180);
  x_temp = r * cos(th);
  z_temp = r * sin(th);

  // angular transform on y - z plane
  // r = sqrt((y_temp*y_temp) + (x_temp*x_temp));
  // th = atan(y_temp/x_temp); 
  // th = th + (90*M_PI/180);
  // x_temp = r * cos(th);
  // y_temp = r * sin(th);

  x_temp = x_temp;
  y_temp = y_temp + 0.3;
  z_temp = z_temp;

  white_pose.pose.position.x = x_temp;
  white_pose.pose.position.y = y_temp;
  white_pose.pose.position.z = z_temp;

  transform_pub.publish(white_pose);
  marker.pose = white_pose.pose;

  p.x = white_pose.pose.position.x; // backward - forward
  p.y = white_pose.pose.position.y; // right - left
  p.z = white_pose.pose.position.z; // down - up

  line.lifetime = ros::Duration(5);
  points.lifetime = ros::Duration(5);

  line.pose.orientation.w = 1.0;
  points.pose.orientation.w = 1.0;
  line.points.push_back(p);
  points.points.push_back(p);
}

void 
transformer_black (const geometry_msgs::PoseStamped& sPose)
{

  float r, th;
  float x_temp = sPose.pose.position.x;
  float y_temp = sPose.pose.position.y;
  float z_temp = sPose.pose.position.z;

  // angular transform on x - y plane
  r = sqrt((z_temp*z_temp) + (x_temp*x_temp));
  th = atan(z_temp/x_temp);
  th = th + (90*M_PI/180);
  x_temp = r * cos(th);
  z_temp = r * sin(th);

  // angular transform on y - z plane
  // r = sqrt((y_temp*y_temp) + (x_temp*x_temp));
  // th = atan(y_temp/x_temp);  
  // th = th - (90*M_PI/180);
  // x_temp = r * cos(th);
  // y_temp = r * sin(th);

  x_temp = x_temp;
  y_temp = y_temp - 0.3;
  z_temp = z_temp;


  black_pose.pose.position.x = x_temp;
  black_pose.pose.position.y = y_temp;
  black_pose.pose.position.z = z_temp;

  transform_pub.publish(black_pose);
  marker.pose = black_pose.pose;

  p.x = black_pose.pose.position.x; // backward - forward
  p.y = black_pose.pose.position.y; // right - left
  p.z = black_pose.pose.position.z; // down - up

  line.lifetime = ros::Duration(5);
  points.lifetime = ros::Duration(5);

  line.pose.orientation.w = 1.0;
  points.pose.orientation.w = 1.0;
  line.points.push_back(p);
  points.points.push_back(p);


}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "marker_transform");
  ros::NodeHandle nh;

  // Initialize marker
  marker_init();
  int count = 0;

  // Create a ROS subscriber for raw cock pose
  ros::Subscriber white_sub = nh.subscribe ("cock_pose_black", 1, transformer_white);
  ros::Subscriber black_sub = nh.subscribe ("cock_pose_white", 1, transformer_black);
  
  transform_pub = nh.advertise<geometry_msgs::PoseStamped>("transformed_pose", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  move_pub = nh.advertise<geometry_msgs::Twist>("read_velocity", 1);

  while(nh.ok()) {
    if(p.x != last.x) {
      if(count > 50) {
        line.points.clear();
        points.points.clear();
        count = 0;
      }
      count++;

      marker_pub.publish(marker);
      // marker_pub.publish(line);
      // marker_pub.publish(points);

    }
    last = p;

    if(marker.pose.position.x > 0.20 && marker.pose.position.y > 0.20) {
      move.linear.x = 1;
      move.linear.y = 1;
    }
    else if(marker.pose.position.x > 0.20) {
      move.linear.x = 1;
    }
    else if(marker.pose.position.y > 0.20) {
      move.linear.y = 1;
    }
    else {
      move.linear.x = 0;
      move.linear.y = 0; 
    }

    // Spin
    ros::spinOnce();
  }
}
