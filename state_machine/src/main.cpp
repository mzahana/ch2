#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/image_encodings.h>
//OpenCV related classes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//Vision classes
#include "ModeSelect.cpp"
#include "CascadeDet.cpp"
#include "ValveDet.cpp"


using namespace std;
using namespace cv;



//Main (Highest level) function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle node_Sm;

	//Mode select class
	//Manages current mode based on state/information flow
	ModeSelect modeObj;
	int mode_State(0);
	

	//Cascade detection object
	CascadeDet cascadeObj;
	cascadeObj.trainNN();

    //Valve detection object
    ValveDet valveObj;


    //Publish/Subscribe
	image_transport::ImageTransport it(node_Sm);
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, &CascadeDet::imageCallback, &cascadeObj);
	ros::Publisher pixelDiff_pub = node_Sm.advertise<geometry_msgs::Twist>("pixelDiff", 1000);
    

	ros::Rate r(5);
	while(ros::ok())
	{
		mode_State = modeObj.modeSwitch(valveObj.radiusValve);
		switch(mode_State){
			case 0:
			valveObj.detect_valve(cascadeObj.frame, cascadeObj.frame_gray);
			case 1:
			cascadeObj.detectProcess();
		}

		//Publish variables to UR5
		pixelDiff_pub.publish(cascadeObj.xyzPxMsg);
		cout << "MAIN: pixelDiff: x[" << cascadeObj.xyzPxMsg.linear.x << "]  y[" << cascadeObj.xyzPxMsg.linear.y <<"]" << endl;
        cout << "MAIN: mode: " << mode_State << endl;
        ros::spinOnce();
		r.sleep();
	}

	cv::destroyAllWindows();
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
}
