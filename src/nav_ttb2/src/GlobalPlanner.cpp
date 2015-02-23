// Running trough ROS
#include <ros/ros.h>
#include <iostream>
#include <sstream>
//OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "stdio.h"
#include "stdlib.h"
#include <nav_msgs/Odometry.h>
#include <pthread.h>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <math.h> 
#include <string>
#include <vector>
#include <math.h>
#include <time.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;

/*Global Variables*/

Mat Map,MapCopy,waypoints,Costgrid,delta,Cost;
int cost=1,rs,cs,row,col,value,rO,goalx,goaly,wp,wpc;
bool click=0;
 
/* Function Headers */
 
void genrute(Mat& init,Mat& goal){

	int cont=0,loc=0,pos=0,a1,b1,ind=0;
	float mini=0, m1=0,m2=0,euclid=0;
 
   	Mat OpenListRute = Mat::zeros(8,3, CV_32F);
	Mat Trayec = Mat::zeros(200,2, CV_32F);	
	Mat TakeListP = Mat::zeros(1,2, CV_32S);
	TakeListP.at<int>(0,0)=goal.at<int>(0,0);
	TakeListP.at<int>(0,1)=goal.at<int>(0,1);

	a1=goal.at<int>(0,0);
	b1=goal.at<int>(0,1);

	for (int i=0;i<8;i++){
    	OpenListRute.at<float>(i,0)=10000;}


	for (int lp=0;lp<2;lp++){
		cont=0;
	   for(int j=0;j<8;j++){
			int m=a1+delta.at<int>(j,0);
       	int n=b1+delta.at<int>(j,1);
 
	 		if ( m==init.at<int>(0,0) && n==init.at<int>(0,1)){
         	cont=1;
				cout<<"****** Path Constructed !!!! :D"<<endl;
           
          	Map.at<uchar>(m,n)=0;
				j=20;
          	lp=2;
            cout << " Trayectory" << endl << " " << Trayec << endl << endl;

            for (int i=1;i<190;i++){ 
            	if (Trayec.at<float>(i,0)!=0 && Trayec.at<float>(i+1,0)!=0){
               	m1=round(100*(Trayec.at<float>(i,1)-Trayec.at<float>(i-1,1))/(Trayec.at<float>(i,0)-Trayec.at<float>(i-1,0)));
                  m2=round(100*(Trayec.at<float>(i+1,1)-Trayec.at<float>(i,1))/(Trayec.at<float>(i+1,0)-Trayec.at<float>(i,0)));
                  if (m1 != m2 ){
                  	waypoints.at<float>(cont,0)=Trayec.at<float>(i,0);
                    	waypoints.at<float>(cont,1)=Trayec.at<float>(i,1);
                     cont=cont+1;
							m1=Trayec.at<float>(i,0);
      					m2=Trayec.at<float>(i,1);  
                     if (cont>0){
              		   	m1=waypoints.at<float>(cont-1,0)-waypoints.at<float>(cont-2,0);
      						m2=waypoints.at<float>(cont-1,1)-waypoints.at<float>(cont-2,1);
                        euclid = sqrt(m1*m1 + m2*m2);
                        	if (euclid<7){cont=cont-1;}}}}}

  				waypoints.at<float>(0,0)=goal.at<int>(0,0);
          	waypoints.at<float>(0,1)=goal.at<int>(0,1);
          	m1=goal.at<int>(0,0);
				m2=goal.at<int>(0,1);
				
				cout << " Wayp " << endl << " " <<waypoints  << endl << endl;
 		
            for (int i=0;i<20;i++){
       			if (waypoints.at<float>(i,0)!=0 && waypoints.at<float>(i,1)!=0){
 						wp=wp+1;
			   		m1=waypoints.at<float>(i,0);
      				m2=waypoints.at<float>(i,1);
					for (int k=-1;k<=1;k++){
						for (int z=-1;z<=1;z++){
					   	Map.at<uchar>(m1+k,m2+z)=0;}}
           		waypoints.at<float>(i,0)=(m1-25)/10;
   				waypoints.at<float>(i,1)=(m2-25)/10;}}

					
				wp=wp-1;
          	cout << " Wayp " << endl << " " <<waypoints  << endl << endl;
            imwrite( "/home/mikelf/Pictures/astarP.jpg", MapCopy );
           
          	break;}

       
     		else if (m>=0 && n>= 0 && m<row && n<col && Costgrid.at<float>(m,n) > 0  ){
  				OpenListRute.at<float>(cont,0)= Costgrid.at<float>(m,n);
          	OpenListRute.at<float>(cont,1)=m;
          	OpenListRute.at<float>(cont,2)=n;
          	cont=cont+1;
          	lp=0;}} 
			
			cont=0;
			mini = OpenListRute.at<float>(0,0);
			loc = 0;
    		for ( pos = 1 ; pos < 8 ; pos++ ){
       		if ( OpenListRute.at<float>(pos,0) < mini && OpenListRute.at<float>(pos,0) >= 0) {
            	mini = OpenListRute.at<float>(pos,0);
	     			loc = pos;}}

   		TakeListP.at<int>(0,0)=OpenListRute.at<float>(loc,1);
   		TakeListP.at<int>(0,1)=OpenListRute.at<float>(loc,2);

   		a1= TakeListP.at<int>(0,0);
   		b1= TakeListP.at<int>(0,1);

  			Trayec.at<float>(ind,0)=TakeListP.at<int>(0,0);
  			Trayec.at<float>(ind,1)=TakeListP.at<int>(0,1);
          
  			ind=ind+1;
   		Map.at<uchar>(a1,b1)=0;
   		MapCopy.at<uchar>(a1,b1)=122;
  
		 	for (int i=0;i<8;i++){
	    		OpenListRute.at<float>(i,0)=100000;}}
}


void AStarSearch(Mat& init,Mat& goal) {
	
 	Mat HeuristicF = Mat::zeros(row,col, CV_32F); 
  	Mat Altergrid = Mat::zeros(row,col, CV_32S);
	Mat TakeList = Mat::zeros(1,2, CV_32F);

  	cv::Size r = delta.size();
  	rs = r.height;
  	cs = r.width;
 
  	Mat OpenList = Mat::zeros(400,4, CV_32F);
  	Mat OpenListTemp = Mat::zeros(400,4, CV_32F);
  
  	cv::Size d = OpenList.size();
  	rO = d.height;
  

	int cont=0,counter=8,it=0,m=0,n=0,ind=0;
	float a=0,b=0,c=0,minimum=0,minimumAux=0,location=0,costref=0;
	
	bool search=1;

	for (int i=0;i<row;i++){
  		for (int j=0;j<col;j++){
   		Costgrid.at<float>(i,j)=1000000;
   		HeuristicF.at<float>(i,j)=max(abs(goal.at<int>(0,0)-i),abs(goal.at<int>(0,1)-j));}}

	a = init.at<int>(0,0);
	b = init.at<int>(0,1);
	Altergrid.at<int>(a,b)=1;
	Costgrid.at<float>(a,b)=0;
 
 	OpenList.at<float>(0,0)=0;
 	OpenList.at<float>(0,1)=a;
 	OpenList.at<float>(0,2)=b;
 	OpenList.at<float>(0,3)= HeuristicF.at<float>(a,b);
 	cont=cont+1;

	while ( search ){
		it=it+1;
		cont=0;

		//Consider actual open list values
   	for(int j=0;j<rO;j++){  
        	if (OpenList.at<float>(j,3)!=0){
         	OpenList.at<float>(j,0)=OpenList.at<float>(cont,0);
          	OpenList.at<float>(j,1)=OpenList.at<float>(cont,1);
          	OpenList.at<float>(j,2)=OpenList.at<float>(cont,2);
          	OpenList.at<float>(j,3)=OpenList.at<float>(cont,3);
            cont=cont+1;}}
 	
		//Compute fist open list
 
		for(int j=0;j<rs;j++){
      	m=a+delta.at<int>(j,0);
        	n=b+delta.at<int>(j,1);
        	value = Map.at<uchar>(m,n);
       
        	if (m>=0 && n>= 0 && m<row && n<col && value == 255 && Altergrid.at<int>(m,n)==0){
		  		MapCopy.at<uchar>(m,n)=200;	
           	cost=Costgrid.at<float>(a,b)+Cost.at<float>(0,j); 
		    	ind=0;  ///Already in the list???
		    	for(int i=0;i<rO;i++){
      	 		if (OpenList.at<float>(i,1)==m && OpenList.at<float>(i,2)==n ){
			      	if (cost<Costgrid.at<float>(m,n)){
			          	Costgrid.at<float>(m,n)=cost;} 
			     		for (int z=0;z<4;z++){
				 			OpenList.at<float>(i,z)=0;}  
               	ind=1;
             		break;}}   
               if (ind==0){
                  if (cost<Costgrid.at<float>(m,n)){
			          	Costgrid.at<float>(m,n)=cost;}   
                	ind=0;} 
              
          	OpenList.at<float>(cont,0)=Costgrid.at<float>(m,n);
          	OpenList.at<float>(cont,1)=m;
          	OpenList.at<float>(cont,2)=n;
          	OpenList.at<float>(cont,3)=Costgrid.at<float>(m,n)+HeuristicF.at<float>(m,n);
          	cont=cont+1;}}
   
	cont=0;

    
 
	//Reorder
	/**********************************/     


	///Make a copy of order list
	for(int i=0;i<rO;i++){
		for(int j=0;j<4;j++){
     		OpenListTemp.at<float>(i,j)= OpenList.at<float>(i,j); }}

   OpenList = Mat::zeros(rO,4, CV_32F);

	//Reorder OpenList
	cont=0;
	counter=0;
 	for(int i=0;i<rO;i++){
   	if (i+1==rO){
			OpenList.at<float>(i,0)=0;
	     	OpenList.at<float>(i,1)=0;
	     	OpenList.at<float>(i,2)=0;
	     	OpenList.at<float>(i,3)=0;
	     	break;}
     else{
  		   if (OpenListTemp.at<float>(i,3) != 0){
         	counter=counter+1;
		  		for(int j=0;j<4;j++){
	      		OpenList.at<float>(cont,j)= OpenListTemp.at<float>(i,j);}   
		 		cont=cont+1;}}}


/****************************//////
 

	for(int j=0;j<rO;j++){ 

		a = OpenList.at<float>(j,1); 
	 	b = OpenList.at<float>(j,2);
	 		if ( a==goal.at<int>(0,0) && b==goal.at<int>(0,1)){
				cout<<"****** Optimally path found !!!! :D"<<endl;
       		genrute(init,goal);
          	cout<<"Iter:    "<<it<<endl;//"#$%&    
				j=rO+20;
          	search=0;}
          else{search=1;}}


	if (OpenList.at<float>(0,0) == 0 && OpenList.at<float>(1,0)== 0 && OpenList.at<float>(2,0)== 0){
		cout<<"****** There is no solution!!!! :("<<endl;
		break;}

	minimum = OpenList.at<float>(0,3);
	location = 0;  

   for ( c = 1 ; c < rO ; c++ ){  
		if ( OpenList.at<float>(c,3) < minimum && OpenList.at<float>(c,3)!=0 ) {
			minimum = OpenList.at<float>(c,3);
     		location = c;}}

	a = OpenList.at<float>(location,1); 
   b = OpenList.at<float>(location,2);
   Altergrid.at<int>(a,b)=1;
 
  	if ( a==goal.at<int>(0,0) && b==goal.at<int>(0,1) ){	
		cout<<"****** Optimally path found !!!! :D"<<endl;} 

  
	//Remove chosen node
  	for (int z=0;z<4;z++){
		OpenList.at<float>(location,z)=0;} 

///Make a copy of order list
	for(int i=0;i<rO;i++){
   	for(int j=0;j<4;j++){
     		OpenListTemp.at<float>(i,j)= OpenList.at<float>(i,j);}}

	OpenList = Mat::zeros(rO,4, CV_32F);
	//Reorder OpenList
	cont=0;
	for(int i=0;i<rO;i++){
		if (i+1==rO){
			OpenList.at<float>(i,0)=0;
	     	OpenList.at<float>(i,1)=0;
	     	OpenList.at<float>(i,2)=0;
	     	OpenList.at<float>(i,3)=0;
	     	break;}
     	else{     
			if (OpenListTemp.at<float>(i,3) != 0){
		  		for(int j=0;j<4;j++){
      			OpenList.at<float>(cont,j)= OpenListTemp.at<float>(i,j);}   
            cont=cont+1;}}}

}}

 
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array){
	
	int Arr[2];
   Mat goal = Mat::zeros(1,2, CV_32S);
   Mat init = Mat::zeros(1,2, CV_32S);
  	init.at<int>(0,0)=25;
  	init.at<int>(0,1)=25;

	if (click == 0){
		int i = 0;
		// save
		for(int j = 0; j <2 ; j++){
			Arr[i] = array->data[j];
			i++;}

		for(int j = 0; j < 2; j++){
			printf("%d, ", Arr[j]);}
 		
		printf("\n");
		goal.at<int>(0,0)=Arr[1];
 		goal.at<int>(0,1)=Arr[0];
  		goalx=goal.at<int>(0,0);
  		goaly=goal.at<int>(0,1);
  		value = Map.at<uchar>(goalx,goaly);
  		AStarSearch(init,goal);
     	click=1;}
	else {ROS_ERROR("Trajectory computed... ");}}


void PlanningCallback(const nav_msgs::Odometry::ConstPtr& msg){

	float xPos;
	float yPos;
	float x_waypnxt;
	float y_waypnxt;
	float angle;
	float distcurr;
	float distnext;
	float xnext;
	float ynext;
	float thetaprev;

	if (click==1){
		cout << " Wp# " << endl << " " <<wp  << endl << endl;
		nav_msgs::Odometry OdomGoal;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
		ros::NodeHandle n; 
 		ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_goal", 30);
		ros::Rate r(1.0);
      ros::Time ahora;
  		ros::Time current_time, last_time;
  		current_time = ros::Time::now();
  
  		float total_distance=0,x_wayp=0,y_wayp=0,theta_wayp=0;
   	float d=0;
 
   	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   	tf::Matrix3x3 m(q);
   	double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
    
    	OdomGoal.header.stamp = current_time;
    	OdomGoal.header.frame_id = "Odom_Goal";

 		//set the position
    	OdomGoal.pose.pose.position.x = 0;
    	OdomGoal.pose.pose.position.y = 0;
   
    	OdomGoal.pose.pose.orientation = odom_quat;

    	//set the velocity
    	OdomGoal.child_frame_id = "goal_orientation";
    	OdomGoal.twist.twist.linear.x = 0;
    	OdomGoal.twist.twist.linear.y = 0;
    	OdomGoal.twist.twist.angular.z = 0;

    //publish the message
 
		xPos=msg->pose.pose.position.x;
 	   yPos=msg->pose.pose.position.y;
	   angle=yaw;

 		x_wayp=(waypoints.at<float>(wpc+wp,0));
 		y_wayp=(waypoints.at<float>(wpc+wp,1));
 		distcurr=sqrt((x_wayp-xPos)*(x_wayp-xPos)+(y_wayp-yPos)*(y_wayp-yPos));
  		
		if ((wpc+wp)>0){ 
 			d=0;
 			x_waypnxt=(waypoints.at<float>(wpc+wp-1,0));
 			y_waypnxt=(waypoints.at<float>(wpc+wp-1,1));
 			distnext=sqrt((x_waypnxt-xPos)*(x_waypnxt-xPos)+(y_waypnxt-yPos)*(y_waypnxt-yPos));} 
 		else{
			d=-0.2;
 			distnext=100*distcurr;
 			x_waypnxt=100*(waypoints.at<float>(wpc+wp,0));
 			y_waypnxt=100*(waypoints.at<float>(wpc+wp,1));}

		OdomGoal.twist.twist.linear.x = wpc+wp;

		std::cout << "wpnumb: "<< wpc+wp << std::endl;
		std::cout << "wpnumb: "<< waypoints << std::endl;

		theta_wayp= atan2(y_wayp-yPos,x_wayp-xPos);

		if (theta_wayp>3 && thetaprev <0){theta_wayp=-3;}
		else if (theta_wayp<-3 && thetaprev > 0){theta_wayp=3;}

		thetaprev=theta_wayp;

  		OdomGoal.pose.pose.position.z = theta_wayp;  

		std::cout << "Desired:  angle_way: "<< theta_wayp <<"  Xway  " <<x_wayp <<"  Yway  " <<y_wayp << std::endl;
		std::cout << "Now:      angle:  "<< angle <<"  Xpos  " <<xPos <<"  Ypos  " <<yPos <<std::endl;
 
 		if ((wpc+wp)<0){ 
  			ROS_WARN("Navigation complete!!!.. ");
  			ros::shutdown();} 

		
		if ( distcurr<0.6+d || (distnext-0.6)<distcurr){
			if (distnext<0.6+d){ 
				wpc=wpc-1;
				ROS_INFO("wp reached!!!.. ");}
			else{
				wpc=wpc-1;
				ROS_INFO("Nearer wp founded!!!.. ");}}

 		OdomGoal.pose.pose.position.x = x_wayp;
 		OdomGoal.pose.pose.position.y = y_wayp;
	 	OdomGoal.pose.pose.position.z = theta_wayp;  	

		odom_pub.publish(OdomGoal);}

	else {ROS_WARN("Waiting for goal... ");}}

int
main (int argc, char** argv){
 
  	wp=0;
 	wpc=0;
  	Map = imread( "/home/mike/Pictures/Kin_Image.jpg", 0 ); //Select the location of the working area map
  	MapCopy = imread( "/home/mike/Pictures/Kin_Image.jpg", 0 );//Select the location of the working area map

  	cv::Size l = Map.size();
  	row = l.height;
  	col = l.width;

   delta = Mat::zeros(8,2, CV_32S);
  	delta.at<int>(0,0)=-1;
  	delta.at<int>(0,1)=0;
  	delta.at<int>(1,0)=0;
  	delta.at<int>(1,1)=-1;
  	delta.at<int>(2,0)=1;
  	delta.at<int>(2,1)=0;
  	delta.at<int>(3,0)=0;
  	delta.at<int>(3,1)=1;
  	delta.at<int>(4,0)=-1;
  	delta.at<int>(4,1)=1;
  	delta.at<int>(5,0)=-1;
  	delta.at<int>(5,1)=-1;
  	delta.at<int>(6,0)=1;
  	delta.at<int>(6,1)=-1;
  	delta.at<int>(7,0)=1;
  	delta.at<int>(7,1)=1;

   Cost= Mat::zeros(1,8, CV_32F);
  	Cost.at<float>(0,0)=1;
  	Cost.at<float>(0,1)=1;
  	Cost.at<float>(0,2)=1;
  	Cost.at<float>(0,3)=1;
  	Cost.at<float>(0,4)=1.4;
  	Cost.at<float>(0,5)=1.4;
  	Cost.at<float>(0,6)=1.4;
  	Cost.at<float>(0,7)=1.4;

   Costgrid = Mat::zeros(row,col, CV_32F);

	ROS_WARN("Loading map... ");
  	
   waypoints= Mat::zeros(20,2, CV_32F);

	//Shows start position 
   for(int i=-3;i<3;i++){
      for(int j=-3;j<3;j++){
         MapCopy.at<uchar>(25+i,25+j)=122;}}
 
   ROS_ERROR("Inflating obstacles... ");
	for (int i=0;i<row;i++){
  		for (int j=0;j<col;j++){
    		if (MapCopy.at<uchar>(i,j)==0){
      		for (int m=-2;m<=2;m++){
          		for (int n=-2;n<=2;n++){
            		Map.at<uchar>(i+m,j+n)=0;}}}}} 

   ros::init (argc, argv, "GP_Navigation");
   ros::NodeHandle n; 
   ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_goal", 10);
   ros::Subscriber sub = n.subscribe("/odom", 10, PlanningCallback);
	ros::Subscriber sub3 = n.subscribe("array", 100, arrayCallback);
	ros::spin();
   return 0;}


