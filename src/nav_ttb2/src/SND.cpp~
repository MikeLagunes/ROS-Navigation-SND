// Running trough ROS
#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <sstream>
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
#include <string>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry,nav_msgs::Odometry > CamSyncPolicy;

using namespace cv;
using namespace std;
using namespace sensor_msgs;
   
	
/* Function Headers */
//This functions were substracted from the article* 
double distc (double alpha, double beta){
   return fmod(alpha-beta,2*M_PI);}

double distcc (double alpha, double beta){
   return fmod(beta-alpha,2*M_PI);}

double dist (double alpha, double beta){
   return fmin(distc(alpha,beta),distcc(alpha,beta));}

double proj(double theta){
   return fmod(theta+M_PI,2*M_PI)-M_PI;}

double sat(int a, int b, double x){
   if (x<=a){x=a;}
   else if (x>=b){x=b;}
   else{x=x;}
   return x;}

double bisec_rob( float s) {//This function consider only the 60 degree range vision from the kinect sensor.
   int n=384;
   return M_PI-(2*s*M_PI/n);}

//Positive angles less than π are on the left of the robot, negative on the right.

 
void Callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg, const nav_msgs::Odometry::ConstPtr& goaldir,const nav_msgs::Odometry::ConstPtr& currodom){
   ros::NodeHandle n; 
   ros::Publisher cmdvelPub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);//This algorithm will publish the command velocities for the turtlebot
   ros::Rate loop_rate(10); // At a frecuancy of 50 Hz (recomendation in ROS answers)
   ros::Time ahora;


   //Turtlebot variables
   geometry_msgs::Twist nav;
   float linearVel,angularVel;
   float Nowx,Nowy;
   float Goalx,Goaly;
   double roll, pitch, yaw;
   int wpnum=0;

   //SND Variables
   float R = 0.4;//0.5
   float Ds = 1.5*R;
   float Drg = 1.5*R;
   float maxV=0.21;//25   0.15 sim
   float maxW=0.4; //6  0.4 sim
   float lrgap=0,comp1=0,comp2=0,WPLength;
   float Tsrg=0,Tmid=0,Trg=0,Tog=0,sTot=0,DeltaAvoid=0;
   float DirGoal,DirNow,DirSND;
   float lim1,lim2;

   //Auxiliary variables
   float aux,compmem,Tempo,obs1,obs2,obs3;
   int i=0,j=0,k=0,DrgAux=0,mem=0,valnum=0;
   bool wpt_obs=0;


   //SND Vectoros
   Mat deltaF;
   deltaF = Mat::zeros( 1, 64, CV_32FC1 ); //This vector saves the depth value in each sector...64 sectors,each one covers 0.93 degrees ...* this variable is used in ND navigation algorithm, in this case is the same that PND ***

   Mat PND;
   PND =  Mat::zeros( 1, 64, CV_32FC1 ); //This vector saves the depth value in each sector

   Mat Gaps;
   Gaps =  Mat::zeros( 1, 30, CV_32FC1 );//This vector saves the index of one sector that contains a gap

   Mat LR;
   LR =  Mat::zeros( 1, 30, CV_32FC1 );// This vector saves if the founded gap is left gap or right gap 

   Mat Regs;
   Regs =  Mat::zeros( 20, 2, CV_32FC1 ); //This vector saves the angles that comforms each region

   Mat RegsLR;
   RegsLR =  Mat::zeros( 20, 2, CV_32FC1 ); // This vector saves if the limits of the region are left o rigth gap

   Mat Valleys;
   Valleys =  Mat::zeros( 20, 6, CV_32FC1 ); // This vector save the founded valleys 

   Mat s; 
   s =  Mat::zeros( 1, 64, CV_32FC1 ); //This term helps to compute the smoothness direction to the goal

   Mat deltaI;
   deltaI =  Mat::zeros( 1, 64, CV_32FC1 );//This term helps to compute the smoothness direction to the goal

   for (int x=0;x<20 ;x++) {Valleys.at<float>(x,4)=100;} // the fifth colum in Valleys saves the minimun angular distance to the desired direction, I initialize this column with 100 to avoid the 0 distance problem*

   Mat BestValley;
   BestValley =  Mat::zeros( 1, 6, CV_32FC1 );//This vector saves all the information (limits,index of gaps, etc,) of the best valley


   //Compute actual direction (read the quaternion's orientation and compute a degree orientation)
   tf::Quaternion q(currodom->pose.pose.orientation.x, currodom->pose.pose.orientation.y, currodom->pose.pose.orientation.z, currodom->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   m.getRPY(roll, pitch, yaw);

   Nowx=currodom->pose.pose.position.x;
   Nowy=currodom->pose.pose.position.y;

   Goalx=goaldir->pose.pose.position.x; //Goal Direction
   Goaly=goaldir->pose.pose.position.y; //Goal Direction

   WPLength=sqrt((Goaly-Nowy)*(Goaly-Nowy)+(Goalx-Nowx)*(Goalx-Nowx)); //euclidian distance to waypoint, this value is requiere for creating artificial valleys


   DirNow=yaw; //Actual Direction
   DirGoal=proj(goaldir->pose.pose.position.z-DirNow); //Goal Direction, referenced at robot's frame +***********
   wpnum=goaldir->twist.twist.linear.x;
   std::cout << "dirGOal: "<< DirGoal << std::endl;
   std::cout << "WPLength: "<< WPLength<< std::endl;

   Gaps.at<float>(0,j)=160; // Limit Gap in the sector 1
   LR.at<float>(0,j)=1;   // First gap is left gap for convenience 
   j=1;

  //Check for the waypoint within range vision. 

  //Fist step to know if the goal is between the robot and an obstacle, I save the gap index with close value to goal direction
	
   k = round(384*(DirGoal-M_PI)/(-2*M_PI))+1;
//cout << " k: "<< k << std::endl;
   if (k>160 && k<224) {k = k-160;}
   else {k = -1;}
 //aux= deltaF.at<float>(0,k);
//cout << " k: "<< k << std::endl;
//cout << " WPLength "<< aux << std::endl;
 
   for ( int x=63;x>=0 ;x--){//This cycle will check the 64 values from the kinect, will check if there is a discontinuity (gap) and what type of gap is (left:from near value to far value   right:from far value to near value *see presentation)

        // Substituing Nan from the kinect (objects out the kinect's range) values for 100 (100 instead 0 to avoid a posible zero division)
      //  if (x==0){cout << " scan 0: "<< scan_msg->ranges[10*x+6]  << std::endl;;}

	if (std::isnan(scan_msg->ranges[10*x])){
          // cout << " 10*x: "<< 10*x << std::endl; 
           if (x==0) {aux=1;}
           else {aux=-1;}
           
          
           deltaF.at<float>(0,i)=scan_msg->ranges[10*x+aux];
               while (std::isnan(scan_msg->ranges[10*x+aux]) && aux<30 && aux>-30)
                     { 
		    //  ROS_ERROR("counting ...!!"); 
                   //   cout << " aux: "<< aux << std::endl;
                      if (aux==-29 || (10*x+aux)<0){aux=1;}
                      else if ( (10*x+aux)>640){aux=30;}
                      if (std::isnan(scan_msg->ranges[10*x+aux]) && aux<0){
                          aux=aux-1;    }
                         // ROS_ERROR("improving ...!!");} 
                      else if (std::isnan(scan_msg->ranges[10*x+aux]) && aux>0){
                          aux=aux+1;
                          }
                      if (not(std::isnan(scan_msg->ranges[10*x+aux]))){deltaF.at<float>(0,i)=scan_msg->ranges[10*x+aux];
//cout << " 10*x: "<< deltaF.at<float>(0,i) << std::endl;
} 
                     }
           
                if (std::isnan(scan_msg->ranges[10*x+aux]) || deltaF.at<float>(0,i)==0){deltaF.at<float>(0,i)=10;}
               /*if (std::isnan(scan_msg->ranges[10*x+90]) || std::isnan(scan_msg->ranges[10*x-90])){deltaF.at<float>(0,i)=200;}   
           else { 
                   if (std::isnan(scan_msg->ranges[10*x+90])){
                  deltaF.at<float>(0,i)=scan_msg->ranges[10*x-90];}
                   else
                  {
                  deltaF.at<float>(0,i)=scan_msg->ranges[10*x+90];}   
                }*/ 

}
        else {deltaF.at<float>(0,i)=scan_msg->ranges[10*x];}
        // PND value for Nan values
        if (deltaF.at<float>(0,i)==200){
           PND.at<float>(0,i)=200;}
        else{PND.at<float>(0,i)=deltaF.at<float>(0,i);} 
        //Creating artificial values in case the goal is between the robot and an obstacle
        if (i>=k-1 && i<=k+1 && deltaF.at<float>(0,i)>WPLength){
          // cout << " i "<<i << std::endl;
           ROS_ERROR("WP between an obstacle");
           // cout << " ObsLength "<< deltaF.at<float>(0,k) << std::endl;
	   PND.at<float>(0,i)=200; 
           wpt_obs=1;}	 
        //Gap finder 
        if (x<63) { 
           lrgap=PND.at<float>(0,i-1)-PND.at<float>(0,i);
	   if (abs(lrgap)>2*R){ 
              Gaps.at<float>(0,j)=i+160; 
              if (lrgap > 0) {LR.at<float>(0,j)=2;}//Right gap condition, #2 for right gap
              else {LR.at<float>(0,j)=1;}//Left gap condition
              j=j+1;}}            
           i=i+1;}
   //cout << " ObsLength "<< deltaF.at<float>(0,k) << std::endl;
   //std::cout << "pnd: "<< PND << std::endl;
   Gaps.at<float>(0,j)=160+64;
   LR.at<float>(0,j)=2; //last gap is right for convenience
   //Ordering the information in regions 
   i=0;
   j=0;
   k=0;
   Regs.at<float>(i,j)=160;
   RegsLR.at<float>(i,j)=1;
   j=1;
   for (int x=1;x<20 ;x++){ 
      if (Gaps.at<float>(0,x)!=0){ 
         Regs.at<float>(i,j)=Gaps.at<float>(0,x);
         RegsLR.at<float>(i,j)=LR.at<float>(0,x);
         i=i+1; 
         j=0;
         Regs.at<float>(i,j)=Gaps.at<float>(0,x);
         RegsLR.at<float>(i,j)=LR.at<float>(0,x);;
         j=j+1;}}
   Regs.at<float>(i,j-1)=0;
   RegsLR.at<float>(i,j-1)=0;
   valnum=i;
   //  std::cout << "Regs: "<< Regs << std::endl;
   //  std::cout << "RegsLR: "<< RegsLR << std::endl;
   //Checking for valley condition (at last one gap in appropiate side) 
        
   for (int x=0;x<20 ;x++){   

          obs1=Regs.at<float>(x,0)-160; 
          obs2=Regs.at<float>(x,1)-161;   
          obs3=round((obs2+obs1)/2);  
          cout << " comp1 "<< comp1 << std::endl;
          cout << " obs3 "<< obs3 << std::endl;
      if (RegsLR.at<float>(x,0)==1 && RegsLR.at<float>(x,1)==2 ){//If there are 2 gaps in appropiate side, this code selects the shortest gap to be the only appropiate side
         Valleys.at<float>(k,0)=Regs.at<float>(x,0);
         Valleys.at<float>(k,1)=Regs.at<float>(x,1);         

  	 comp1=abs(DirGoal-bisec_rob(Regs.at<float>(x,0)));
         comp2=abs(DirGoal-bisec_rob(Regs.at<float>(x,1))); 
      
               
       //  if (abs(abs(comp1)-abs(comp2))<0.5 && valnum<2){
          

         if ( valnum==1 && deltaF.at<float>(0,obs3)<1.5 && abs(abs(comp1)-abs(comp2))<1){
            
               comp1 =10-deltaF.at<float>(0,obs1);
               comp2 =10-deltaF.at<float>(0,obs2);
               cout << " comp2 "<< comp2 << std::endl;
               ROS_ERROR("obstacle in front");
          }
	
         if (comp1<comp2){   
            Valleys.at<float>(k,2)=bisec_rob(Regs.at<float>(x,0));
	    Valleys.at<float>(k,3)=bisec_rob(Regs.at<float>(x,1));//0.6
           // if ( valnum==2 && deltaF.at<float>(0,obs3)<10){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}//  }
            if ( valnum==2 && WPLength>0.8 ){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}//  }
	    else{Valleys.at<float>(k,4)=comp1;}
             //value for compute best valley 
            Valleys.at<float>(k,5)=1;
            k=k+1;
            compmem=bisec_rob(Regs.at<float>(x,0));}
         else{Valleys.at<float>(k,2)=bisec_rob(Regs.at<float>(x,1));
	    Valleys.at<float>(k,3)=bisec_rob(Regs.at<float>(x,0));
            //if ( valnum==2){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}
            if ( valnum==2 && WPLength>0.8){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}
            else {Valleys.at<float>(k,4)=comp2;}
            Valleys.at<float>(k,5)=2;
            k=k+1;    
            compmem=bisec_rob(Regs.at<float>(x,1)); }}

      else if (RegsLR.at<float>(x,0)==1 || Regs.at<float>(x,0)==160 && valnum<4 ){ //If there is a left valley      160
         Valleys.at<float>(k,0)=Regs.at<float>(x,0);
         Valleys.at<float>(k,1)=Regs.at<float>(x,1);         
         Valleys.at<float>(k,2)=bisec_rob(Regs.at<float>(x,0));
         Valleys.at<float>(k,3)=bisec_rob(Regs.at<float>(x,1));
         comp1=abs(DirGoal-bisec_rob(Regs.at<float>(x,0)));
         //if ( valnum==2 && deltaF.at<float>(0,obs3)<10){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}
         if ( valnum==2 && WPLength>0.8){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}
         else {Valleys.at<float>(k,4)=comp1;}
         Valleys.at<float>(k,5)=1;
         k=k+1;}
      else if (RegsLR.at<float>(x,1)==2 || Regs.at<float>(x,1)==224 && valnum<4) { //If there is a right valley 224
         Valleys.at<float>(k,0)=Regs.at<float>(x,0);
         Valleys.at<float>(k,1)=Regs.at<float>(x,1);         
         Valleys.at<float>(k,2)=bisec_rob(Regs.at<float>(x,1));
         Valleys.at<float>(k,3)=bisec_rob(Regs.at<float>(x,0));
         comp2=abs(DirGoal-bisec_rob(Regs.at<float>(x,1))); //In all cases I compute the distance to the desired goal     c
         //if ( valnum==2 && deltaF.at<float>(0,obs3)<10){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}
         if ( valnum==2 && WPLength>0.8){Valleys.at<float>(k,4)=10-deltaF.at<float>(0,obs3);}
         else {Valleys.at<float>(k,4)=comp2;}
         Valleys.at<float>(k,5)=2;    
         k=k+1;}} 
  
   i=0;
   j=0;
   k=0;
  /* std::cout << "Vallnum: "<< valnum << std::endl;
   std::cout << "Regs: "<< Regs << std::endl;
   std::cout << "RegsLR: "<< RegsLR << std::endl;
   std::cout << "Valleys: "<< Valleys << std::endl;*/
  ///Looking for best valley
   
   //Select the best valley based on the value in the 4 column (distance to the goal direction)
   comp1=Valleys.at<float>(0,4); 
   comp2=0;

   for ( i=0;i<20 ;i++){ 
      if (Valleys.at<float>(i,4)<comp1){
         comp1=Valleys.at<float>(i,4);
         comp2=i;}}

   //BEst Valley Properties 
   BestValley.at<float>(0,0)=Valleys.at<float>(comp2,2); //angle limit i
   BestValley.at<float>(0,1)=Valleys.at<float>(comp2,3); //angle limit j
   BestValley.at<float>(0,2)=Valleys.at<float>(comp2,5); //left gap o right gap
   BestValley.at<float>(0,3)=Valleys.at<float>(comp2,0); //
   BestValley.at<float>(0,4)=Valleys.at<float>(comp2,1); //
   BestValley.at<float>(0,5)=Valleys.at<float>(comp2,4); //winner

  std::cout << "BestValley: "<< BestValley << std::endl;

  //Once the best valley is selected the next term are necessary to compute the snd direction, this  terms were subtracted from the article 

   if (BestValley.at<float>(0,2)==1){ 
      if (BestValley.at<float>(0,3)==160){
         DrgAux=Valleys.at<float>(comp2,0)-160;
         Drg=PND.at<float>(0,DrgAux);}
      else{DrgAux=Valleys.at<float>(comp2,0)-161;
      Drg=PND.at<float>(0,DrgAux);}}
  else{
      if (BestValley.at<float>(0,4)==224){
         DrgAux=Valleys.at<float>(comp2,1)-161;
         Drg=PND.at<float>(0,DrgAux);}
      else{DrgAux=Valleys.at<float>(comp2,1)-160;
      Drg=PND.at<float>(0,DrgAux);}}
   //DrgAux=Valleys.at<float>(comp2,0)-160;
   //Drg=PND.at<float>(0,DrgAux);}

   Trg=Valleys.at<float>(comp2,2);
   Tog=Valleys.at<float>(comp2,3);

   /* std::cout << "drg: "<< Drg << std::endl;
    std::cout << "Trg: "<< Trg << std::endl;*/
   //std::cout << "Valleys: "<< Valleys << std::endl;
   //Computing desire angle
   
   if (Drg == 200 || BestValley.at<float>(0,3)==160 || BestValley.at<float>(0,4)==224 ){Tsrg=Trg ;}
   else{
      if (Drg<R+Ds){ Drg = R+Ds;}
      if (BestValley.at<float>(0,2) == 1){Tsrg=Trg-asin((R+Ds)/Drg);} //***++****+**
      else{Tsrg=Trg+asin((R+Ds)/Drg);}} 
 
   if (BestValley.at<float>(0,2) == 1){Tmid=Trg-distc(Trg,Tog)/2;} //***++****+**
   else{Tmid=Trg+distcc(Trg,Tog)/2;}

   comp1=bisec_rob(BestValley.at<float>(0,3));
   comp2=bisec_rob(BestValley.at<float>(0,4));


    //std::cout << "Tmid: "<< Tmid << std::endl;
    //std::cout << "Tsrg: "<< Tsrg << std::endl;
   //std::cout << "Tmid: "<< dist(Tmid,Trg) << std::endl;
   //std::cout << "Tsrg: "<< dist(Tsrg,Trg) << std::endl;

   if (DirGoal<comp1 && DirGoal>comp2 && wpt_obs==1){DirSND=DirGoal;
   ROS_ERROR("Dirgoal");
   std::cout << "wpobs: "<< wpt_obs << std::endl;}
   else if ( abs(dist(Tsrg,Trg))<abs(dist(Tmid,Trg)) || valnum==1){DirSND=Tsrg; //*with abs()&& valnum>1
   ROS_ERROR("Tsrg");}
   else{
   DirSND=Tmid;
   ROS_ERROR("Tmid");}


   /*if (DirGoal<comp1 && DirGoal>comp2 ){DirSND=DirGoal;
   ROS_ERROR("Dirgoal");
   std::cout << "wpobs: "<< wpt_obs << std::endl;}
   else if ( abs(dist(Tsrg,Trg))<abs(dist(Tmid,Trg))){DirSND=Tsrg; //*with abs()&& valnum>1
   ROS_ERROR("Tsrg");}
   else{
   DirSND=Tmid;
   ROS_ERROR("Tmid");}*/


   //std::cout << "Selected:  "<< DirSND*180/M_PI <<"  Tsrg  " <<Tsrg*180/M_PI <<"  Tmid " <<Tmid*180/M_PI << std::endl;

//cout << "DirSND:  "<< DirSND*180/M_PI  << std::endl;

 //Reduces max speed when is closer to the goal (only 1 waypoint left)
   if (wpt_obs==1 && wpnum==0) { maxV=0.15;
      Ds=0.25;
      ROS_ERROR("Approaching to goal & safety Ds reduction...!!");}
  


   //Once the snd direction has been obtained, I compute the firsr Obstacle avoidance method approach, which involves the weight sum of each sector, considering the obstacles
   i=0;
   j=0;

   for ( int x=0;x<64 ;x++){  
      	//s_i terms for smooth obstacle avoidance
      if (PND.at<float>(0,x)==200){PND.at<float>(0,x)=10;}
      aux= (Ds+R-PND.at<float>(0,x))/Ds;
      s.at<float>(0,x)=sat(0,1,aux);
      if (x<32){DrgAux=(-M_PI+(0.01623*(32-x)));}
      else {DrgAux=(M_PI-(0.01623*(x-32)));}
       
      //deltaI.at<float>(0,x)=s.at<float>(0,x)*proj(distcc(DrgAux,DirSND));
      deltaI.at<float>(0,x)=s.at<float>(0,x)*DrgAux;
      sTot=sTot+s.at<float>(0,x)*s.at<float>(0,x);}

   std::cout << "delta: "<< deltaI << std::endl;
   //std::cout << "sat: "<< s<< std::endl;
   //s_i terms for smooth obstacle avoidance
   if (sTot==0){DeltaAvoid=0;}
   else{
      for ( int x=0;x<64 ;x++){DeltaAvoid=DeltaAvoid+(s.at<float>(0,x)*s.at<float>(0,x)*deltaI.at<float>(0,x))/sTot;}}

   DirSND=DirSND+DeltaAvoid;

   DeltaAvoid=DeltaAvoid*180/M_PI;
   // std::cout << "deltaid: "<< deltaI << std::endl;
   std::cout << "delta all: "<< DeltaAvoid << std::endl;
   //std::cout << "dimid: "<< Tmid << std::endl;
   //std::cout << "digoal: "<< DirGoal << std::endl;
   std::cout << "dirSND: "<< DirSND*180/M_PI << std::endl;
   //std::cout << "dirnow: "<< DirNow << std::endl;

   //The next code, will select the maximun value in s, in order to decrease the robot's velocity, this is the second approach to smooth and safe navigation
   double minVal; 
   double maxVal; 
   Point minLoc; 
   Point maxLoc;

   minMaxLoc( s, &minVal, &maxVal, &minLoc, &maxLoc );

   std::cout << "max: "<< maxVal << std::endl;
   //std::cout << "max: "<< minVal << std::endl;
   //std::cout << "DSND: "<< DirSND*180/M_PI << std::endl;


  
   std::cout << "deltf: "<< deltaF << std::endl;
   std::cout << "PND: "<< PND << std::endl;
   std::cout << "wp#: "<< wpnum << std::endl;
 //  std::cout << "wp#: "<< waypoints << std::endl;

  

   linearVel = maxV-(maxV*maxVal);

    if (valnum==1 && deltaF.at<float>(0,obs3)<0.5){maxW=0.6;}  
    // if (valnum==wpnum && deltaF.at<float>(0,obs3)<0.5){maxW=0.6;}  


   Tempo=(DirSND/(M_PI/5)); // /4 ok ok 
   angularVel = sat(-1,1,Tempo)*maxW;
   Tempo=(3*M_PI/4-abs(DirSND))/(3*M_PI/4);// /1 ok ok 3/4
   maxV=sat(0,1,Tempo)*linearVel;

   linearVel=maxV;
  std::cout << "linearvel: "<< linearVel << std::endl;
   std::cout << "angulrvel: "<< angularVel << std::endl;
   //linearVel = maxV-(maxV*maxVal);
   //Publish velocity commands
   nav.linear.x = linearVel;
   nav.angular.z = angularVel;
   cmdvelPub.publish(nav); // Finally the algorith publish the information to the robot
   mem=mem+1;}


int
main (int argc, char** argv){
   // Initialize ROS
   ros::init (argc, argv, "SND_Navigation");
   ros::NodeHandle nh;
   ros::NodeHandle n; 
   ros::Publisher cmdvelPub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
   ROS_ERROR("Navigating... START");
   message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 0);
   message_filters::Subscriber<nav_msgs::Odometry> sub_cam_info(nh, "/odom_goal", 0);
   message_filters::Subscriber<nav_msgs::Odometry> odom_turtle_info(nh, "/odom", 0);
   message_filters::Synchronizer<CamSyncPolicy> sync(CamSyncPolicy(1000), scan_sub, sub_cam_info,odom_turtle_info);//40
   sync.registerCallback(boost::bind(&Callback, _1, _2, _3)); //this algorithm requieres the goal direction, the current direction and the laserscan values from the kinect
   ros::spin();
   return 0;}
 

//***:check for further optimization v 2.0
