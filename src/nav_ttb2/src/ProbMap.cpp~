// Running trough ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
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

//OpenCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "stdio.h"
#include "stdlib.h"
#include <nav_msgs/Odometry.h>
#include <pthread.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <fstream>
  
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf/transform_datatypes.h>
#include <math.h> 
#include <ctime>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> CamSyncPolicy;
//using namespace message_filters;
using namespace cv;
using namespace std;
using namespace sensor_msgs;
 

/*Globla Variables*/
int map_sx=200;
int map_sy=200;
ros::Publisher pub;
ros::Time current_time, last_time;

Mat maplog=Mat::zeros( map_sx, map_sy, CV_32F);




/* Function Headers */
 
void  myfunct(Mat& map)
{
namedWindow( "2D Probabilistic Map", WINDOW_AUTOSIZE );// Create a window for display.
  //  imshow( "Display window", image );                   // Show our image inside it.


  imshow("2D Probabilistic Map",map);
  moveWindow( "2D Probabilistic Map", 400,400);
  waitKey(1); //10 best now
  imwrite( "/home/mike/Pictures/Kin_ImageTEST.jpg", map );
  
}

void Callback(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::Odometry::ConstPtr& msg)
{
typedef pcl::PointXYZ point;
typedef pcl::PointCloud<pcl::PointXYZ> cloudin;
typedef cloudin::Ptr cloudinptr;
 // sensor_msgs::PointCloud2 output;
  // Publish the data
  current_time = ros::Time::now();
//const clock_t begin_time = clock();
cloudinptr cloud(new cloudin());
  //current_time = ros::Time::now();
 //std::cout << "time: "<< current_time-last_time<< std::endl;
  //last_time = ros::Time::now();
//std::cout << "BestValley: "<< BestValley << std::endl;
 // std::cout << "initime: "<< current_time<< std::endl;
  
int paso=100; //paso 10cm
int l_free=-4;
int l_occ=12;//return 10
int i_maplog;
int j_maplog;
long int xo =2500;  //check this values 10 000
long int yo=2500;
long int yf=10000;  //check this values 10 000
long int xf =10000;


float xPos;
float yPos;
float angle;
float tempx;
float tempz;
//float fact=1.57;

   float total_distance;
   float dr;
   float v_size;
   float dx;
   float dy;
   int xp;
   int yp;
   int xrel;
   int yrel;
   Mat maplogProb= Mat::zeros( map_sx, map_sy, CV_32FC1 );
   Mat map= Mat::zeros( map_sx,map_sy, CV_8UC1 );
 
//sensor_msgs::PointCloud2 cloud_filtered;


 //  Mat maplogtemp;


  // Mat mapCopy;
   //Mat mapPath;

// maplogProb 
 //maplogtemp = Mat::zeros( map_sx, map_sy, CV_32F );
  //mapCopy= Mat::zeros( map_sx,map_sy, CV_8UC1 ); 
  //mapPath= Mat::zeros( map_sx,map_sy, CV_8UC1 ); 
// map 
  
 /*for (int b=0;b<map_sx;b++)
   {
   for (int c=0;c<map_sy;c++)
    {
    mapCopy.at<uchar>(b,c)=1;
     }
    }

*/
   int  k;
   tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
  //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
    pcl::fromROSMsg(*input, *cloud); //Now you can process this CLloud using the pcl functions 
    xPos=msg->pose.pose.position.x;
    yPos=msg->pose.pose.position.y;
    
    //convert to pitch*/
    angle=-yaw;
    if (angle<0){angle=6.28+angle;};
    //angle=angle-M_PI/3;
 //  std::cout << "x: " << xPos << ", y: " << yPos << ", angle: " << angle << std::endl;
  // cout << "size " <<cloud->points.size () <<endl;
  // Scale *100 to get point cloud data in centimeters
  /* for (size_t i = 0; i < cloud->points.size (); i++)
    {
      cloud->points[i].x=-1000*(cloud->points[i].x);
      cloud->points[i].z=1000*(cloud->points[i].z);
    }*/
 

 
  


for (k=1;k<cloud->points.size (); k++)
 {
// k=k+10;
//cout << "ok2 = " << endl;
//ros::Rate loop_rate(2); // At a frecuancy of 50 Hz (recomendation in ROS answers)
   //ros::Time ahora;
 if (cloud->points[k].z < 8)

{tempx =-1000*cloud->points[k].x ;
tempz = 1000*cloud->points[k].z ;

/*while ( tempz ==0 ) 
{
k=k+1;
tempx =-1000*cloud->points[k].x ;
tempz = 1000*cloud->points[k].z ;
}*/

cloud->points[k].x=(tempx)*cos(angle)-(tempz)*sin(angle);//+(100*xPos)
cloud->points[k].z=(tempx)*sin(angle)+(tempz)*cos(angle);//+(100*yPos)

  
//   cout << "td = " << endl << " " << total_distance << endl << endl;
     total_distance = sqrt((cloud->points[k].x)*(cloud->points[k].x)+(cloud->points[k].z)*(cloud->points[k].z));    
   dr = round(total_distance);
   dx = round((cloud->points[k].z))/dr;
   dy = round((cloud->points[k].x))/dr;
   v_size=dr+1;
//   cout << "xv = " << endl << " " << v_size << endl << endl;
//   xv = Mat::zeros( 1, v_size, CV_32FC1 );
//   yv = Mat::zeros( 1, v_size, CV_32FC1 ); 
   Mat xv=Mat::zeros( 1, v_size, CV_32FC1 );
   Mat yv=Mat::zeros( 1, v_size, CV_32FC1 );
  
   xv.at<float>(0,0)=1000*xPos;   
   yv.at<float>(0,0)=1000*yPos;

  
   for (int c=1;c<= v_size;c++)
   {
   xv.at<float>(0,c)=xv.at<float>(0,c-1)+dx;
   yv.at<float>(0,c)=yv.at<float>(0,c-1)+dy;   
   }

   //Normalize data

    
   for (int c=0;c<= v_size;c++)
   {
   xv.at<float>(0,c)=cvRound(xv.at<float>(0,c)/paso)*paso;
   yv.at<float>(0,c)=cvRound(yv.at<float>(0,c)/paso)*paso;
   }
   
   //Initialize map's matrix
   xp = xv.at<float>(0,0);
   yp = yv.at<float>(0,0);


   

  
   j_maplog = cvRound((yp+yo)/paso); 
   i_maplog = cvRound((xp+xo)/paso);



  // cout << "i = " <<i_maplog<< "j= " << j_maplog << endl << endl;
  
   maplog.at<float>(i_maplog,j_maplog)=maplog.at<float>(i_maplog,j_maplog)+ l_free;
  

   for(int n=1;n<v_size;n++)
    {
     if (xv.at<float>(0,n)!=xp || yv.at<float>(0,n)!=yp )
       {
       xp = xv.at<float>(0,n);
       yp = yv.at<float>(0,n);
       j_maplog = cvRound((yp+yo)/paso); 
   i_maplog = cvRound((xp+xo)/paso);
       maplog.at<float>(i_maplog,j_maplog)=maplog.at<float>(i_maplog,j_maplog)+ l_free;
  //     cout << "i = " <<i_maplog<< "j= " << j_maplog << endl << endl;
       }
     }

   j_maplog = cvRound((yp+yo)/paso);
   i_maplog = cvRound((xp+xo)/paso);
    maplog.at<float>(i_maplog,j_maplog)=maplog.at<float>(i_maplog,j_maplog)-l_free+l_occ;
  
  //k=k+10;
   } //if
  
    
  }	 //for k maplog
 
 
  //pcl::toROSMsg(*cloud_projected, cloud_filtered);
  //cloud_filtered.header.frame_id = cloud->header.frame_id;



 

 
 for (int b=0;b<map_sx;b++)
   {
   for (int c=0;c<map_sy;c++)
    {
    maplogProb.at<float>(b,c)=1-1/(1+exp(maplog.at<float>(b,c))) ;
     }
    }

 


 for (int b=0;b<map_sx;b++)
   {
   for (int c=0;c<map_sy;c++)
    {
     if (maplogProb.at<float>(b,c)==1) //60 |1200
       {
         map.at<uchar>(b,c)=0;
      //   mapCopy.at<uchar>(b,c)=0;
       }
     else if (maplogProb.at<float>(b,c)==0) //|-400
       {
	map.at<uchar>(b,c)=255;

      }
      else
        map.at<uchar>(b,c)=125;
     }
    }


myfunct(map);
last_time = ros::Time::now();

//std::cout << "time: "<< last_time-current_time<< std::endl; 

}

int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "ProbMap");
  ros::NodeHandle nh;
  //maplog = Mat::zeros( map_sx, map_sy, CV_32F );//_32F  , 8UC3 
 ROS_INFO("Mapping Enviroment... ");
 ROS_WARN("Please control the robot manually ");
  
//ros::Rate r(1); // 10 hz
//while (ros::ok())
//{
//ROS_ERROR("Tmid");
 message_filters::Subscriber<sensor_msgs::PointCloud2> sub_img(nh, "cloud_projectedZ", 1);
 message_filters::Subscriber<nav_msgs::Odometry> sub_cam_info(nh, "/odom", 10);
 message_filters::Synchronizer<CamSyncPolicy> sync(CamSyncPolicy(100), sub_img, sub_cam_info);
 
sync.registerCallback(boost::bind(&Callback, _1, _2));
 //TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(sub_img, sub_cam_info, 0);  
// Create a ROS publisher for the output point cloud
 // pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_map", 5);
  //sync.registerCallback(boost::bind(&Callback, _1, _2));
  //... do some work ...
 // r.sleep();
//}
//ros::Rate r(2);
//while (ros::ok())
//{

ros::spin();
//r.sleep();
//}


   //loop_rate.sleep();
  return 0;
}

