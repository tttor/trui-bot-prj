
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

ros::Publisher cloud_pub_white;
ros::Publisher cock_pose_white_pub;

void csv_write(const geometry_msgs::PoseStamped& pose,
               const std::string& filepath) {
  using namespace std;
  using namespace boost;

  ofstream csv;
  csv.open(filepath.c_str(),ios::app);
  if ( csv.is_open() ) {
    // sPose.header.stamp = ros::Time::now();
    csv << lexical_cast<string>(pose.pose.position.x); csv << ",";
    csv << lexical_cast<string>(pose.pose.position.y); csv << ",";
    csv << lexical_cast<string>(pose.pose.position.z); csv << ",";
    csv << lexical_cast<string>(pose.header.stamp.toSec()); csv << ",";

    // if (new_sample)
      csv << "\n";
  }
  else {
    assert(false && "csv.open(filepath.c_str()): FALSE");
  }
  
  csv.close();
}

void 
cloud_cb_white (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment (new pcl::PointCloud<pcl::PointXYZ>);

   // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform downsampling
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // convert PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered,*temp_cloud);

  // segmenting area
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (temp_cloud);
  // left - right
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-3, 3);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_segment);

  pass.setInputCloud (cloud_segment);
  // up - down
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-3, 3);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_segment);

  pass.setInputCloud (cloud_segment);
  // backward - forward
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.5, 4);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_segment);

  // // Find centroid
  float sum_x = 0;
  float sum_y = 0;
  float sum_z = 0;
  if(cloud_segment->points.size() > 50) {
    for(size_t i = 0; i < cloud_segment->points.size(); ++i) {
     sum_x += cloud_segment->points[i].x;
     sum_y += cloud_segment->points[i].y;
     sum_z += cloud_segment->points[i].z;
    }

    sum_x = sum_x / cloud_segment->points.size();
    sum_y = sum_y / cloud_segment->points.size();
    sum_z = sum_z / cloud_segment->points.size();

    geometry_msgs::PoseStamped sPose;
    sPose.header.stamp = ros::Time::now();
    
    sPose.pose.position.x = sum_z; // equals to z in pcl // backward - forward
    sPose.pose.position.y = -(sum_x); // equals to -x in pcl // right - left
    sPose.pose.position.z = -(sum_y); // equals to -y in pcl // down - up

    sPose.pose.orientation.x = 0.0;
    sPose.pose.orientation.y = 0.0;
    sPose.pose.orientation.z = 0.0;
    sPose.pose.orientation.w = 1.0;

    cock_pose_white_pub.publish(sPose);

    // std::string csv_filepath = "/home/lintang-sutawika/krai/tor/data/input/test.csv";
    // csv_write(sPose,csv_filepath);
  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //pcl_conversions::fromPCL(cloud_filtered, output);
  pcl::toROSMsg(*cloud_segment, output);

  // Publish the data
  cloud_pub_white.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_input");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_white = nh.subscribe ("/camera_white/depth/points", 1, cloud_cb_white);


  // Create a ROS publisher for the output point cloud
  cloud_pub_white = nh.advertise<sensor_msgs::PointCloud2> ("output_white", 1);
  cock_pose_white_pub = nh.advertise<geometry_msgs::PoseStamped>("cock_pose_white", 1);

  // Spin
  ros::spin();
}
