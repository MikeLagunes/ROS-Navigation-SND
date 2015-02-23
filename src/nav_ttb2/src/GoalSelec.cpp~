// Running trough ROS
#include <ros/ros.h>
#include <iostream>
#include <sstream> 
//OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "stdio.h"
#include "stdlib.h"
#include <pthread.h>
#include "std_msgs/String.h"
#include <fstream>
#include <tf/transform_datatypes.h>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Int32.h>
#include <GL/glut.h> // for working with mouse clicks
#include <GL/gl.h>
#include "std_msgs/MultiArrayLayout.h"//for send the goal
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
 

/*Globla Variables*/

Mat  Map,MapCopy,waypoints,Costgrid,delta,Cost;
int  cost=1,rs,cs,row,col,value,rO,goalx,goaly,wp,wpc,xmouse,ymouse;
bool click_event;
 

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
            imwrite( "/home/mikelf/Pictures/astarP.jpg", MapCopy ); //Select a desired route on the workstation machine
           
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

   //Make a copy of order list
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


void timerCb() { 

	Mat goal = Mat::zeros(1,2, CV_32S);
   Mat init = Mat::zeros(1,2, CV_32S);
  
   //initializing
   init.at<int>(0,0)=25; //Origin at 25,25 -> (2.5 m, 2.5m) away from the uppers left corner in the map
   init.at<int>(0,1)=25;
   goal.at<int>(0,0)=ymouse;
   goal.at<int>(0,1)=xmouse;
   goalx=goal.at<int>(0,0);
   goaly=goal.at<int>(0,1);
   value = Map.at<uchar>(goalx,goaly);    
   AStarSearch(init,goal);
   ros::NodeHandle n; 
   ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("array", 100);
	ros::Rate r(60);
	while (ros::ok()){
               
		namedWindow( "A* Optimal Route", CV_WINDOW_AUTOSIZE );// Create a window for display.
    	moveWindow("A* Optimal Route", 600,400);
    	imshow( "A* Optimal Route", Map );   
    	waitKey(5);  
    	namedWindow( "Cells Consulted", CV_WINDOW_AUTOSIZE );// Create a window for display.
    	imshow( "Cells Consulted", MapCopy );        
    	moveWindow("Cells Consulted", 800,400);
    	waitKey(5);                // Show our image inside it.
    	ROS_INFO("Publishing desired Goal... ");
   	std_msgs::Int32MultiArray array;
		//Clear array
		array.data.clear();
		//assign array a random number between 0 and 255.
		array.data.push_back(xmouse);
    	array.data.push_back(ymouse);
		  
		//Publish array
		pub.publish(array);
  		r.sleep();}}



void mouse(int button, int state, int x, int y){

	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN && click_event ==0){
		ROS_ERROR("Goal selected ");
      xmouse=x; 
      ymouse=y; 
      click_event=1;}

	if (click_event==1){ glutIdleFunc(timerCb);  }}



int
main (int argc, char** argv){

	click_event=0;
  	Map = imread( "/home/mike/Pictures/Kin_Image.jpg", 0 ); //Select the location of the working area map
  	MapCopy = imread( "/home/mike/Pictures/Kin_Image.jpg", 0 );//Select the location of the working area map
  	cv::Size l = Map.size();
  	row = l.height;
  	col = l.width;

  	ROS_INFO("Loading Map... ");

  	Costgrid = Mat::zeros(row,col, CV_32F);
  	waypoints= Mat::zeros(20,2, CV_32F);



   for(int i=-3;i<3;i++){
   	for(int j=-3;j<3;j++){
      	MapCopy.at<uchar>(25+i,25+j)=122;}   }
 
	ROS_WARN("Inflating obstacles... ");


	for (int i=0;i<row;i++){
  		for (int j=0;j<col;j++){
    		if (MapCopy.at<uchar>(i,j)==0){	
      		for (int m=-2;m<=2;m++){
          		for (int n=-2;n<=2;n++){
            		Map.at<uchar>(i+m,j+n)=0;}}}}	}

   namedWindow( "Select a Goal", CV_WINDOW_AUTOSIZE );// Create a window for display.
   imshow( "Select a Goal", Map ); 
   moveWindow("Select a Goal", 400,400);
  	waitKey(1000);  
 
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

  	cv::Size r = delta.size();
  	rs = r.height;
  	cs = r.width;

   ros::init (argc, argv, "Sending_goal");
   glutInit(&argc, argv);
	ros::NodeHandle n; 


	ROS_WARN("Please Select a goal ... ");

	glutInitDisplayMode ( GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowPosition(400,400);
	glutInitWindowSize(row,col);
	glutCreateWindow ("Select a Goal");
	glutMouseFunc(mouse);
	glutMainLoop(); //GLU libraries will take the main
   return 0;}


