//ROS includes
#include <ros/ros.h>
//ROS mesages includes
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <fstream>
//PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//C++ I/O data includes
#include <iostream>
#include <sstream>
#include "stdio.h"
#include "stdlib.h"


using namespace std;
ros::Publisher pub;

void 
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  //sensor_msgs::PointCloud2 cloud_filtered;
   pcl::PCLPointCloud2 cloud_filtered;
   //pcl::VoxelGridlt;pcl::PCLPointCloud2&gt; sor;
   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  
// Tranform de PC data from Kinect to PCL library format, and perform a voxel filtering 
  //pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
 // sor.setLeafSize (0.04, 0.04, 0.04);
 // sor.setLeafSize (0.1, 0.1, 0.1);
  sor.setLeafSize (0.01, 0.015, 0.01);
  sor.filter (cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "CloudFiltered");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);

  // Spin
  ros::spin ();
}
