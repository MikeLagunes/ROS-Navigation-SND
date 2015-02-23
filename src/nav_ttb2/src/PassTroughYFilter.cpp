// Running trough ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
// PCL cloud 3D to 2D projection includes
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include "pcl_ros/filters/filter.h"
#include <message_filters/subscriber.h>
#include <pcl/filters/passthrough.h>
#include <unistd.h>

typedef pcl::PointXYZ point;
typedef pcl::PointCloud<pcl::PointXYZ> cloudin;
typedef cloudin::Ptr cloudinptr;


using namespace std;
ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  
  cloudinptr cloud (new cloudin());
  cloudinptr cloud_projected (new cloudin());
  pcl::fromROSMsg(*input, *cloud); //Now you can process this CLloud using the pcl functions 
  sensor_msgs::PointCloud2 cloud_filtered;


// Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass(true);
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0,0.01);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_projected);
  pcl::toROSMsg(*cloud_projected, cloud_filtered);
  cloud_filtered.header.frame_id = cloud->header.frame_id;
  for (int i=0; i<900;i++)
  {

usleep(900);

   }
 

  pub.publish (cloud_filtered);
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "secondfilter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud_filtered", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_projectedZ", 1);


  // Spin
  ros::spin ();
}


