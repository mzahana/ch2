#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
//OpenCV related classes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"


//Vision classes
#include "VizLibrary.cpp"
#include "IdentifySize.cpp"


using namespace std;
using namespace cv;



//Vision main function. Manages vizlib, IdentifySize, and TrackTool classes
//Subscribes to M_v, publishes mode_Viz
int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_main");
	ros::NodeHandle node_Viz;
	
	//Cascade detection object
	VizLibrary vizlibObj;
	vizlibObj.trainNN();
	vizlibObj.detectProcess();
	
	/*
	//Mode select class object
	//Manages current mode based on state/information flow
	//Size identification object
	IdentifySize identObj;

	//Tool tracker object
	TrackTool trackObj;
	*/
	
	//Variables published
	std_msgs::Int32 mode_Viz;
	
	//SUBSCRIBE
	//Image transport object
	image_transport::ImageTransport it(node_Viz);
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_rect_color", 1, &VizLibrary::imageCallback, &vizlibObj);
	//System mode
	ros::Subscriber sub_mode_system = node_Viz.subscribe<std_msgs::Int32MultiArray>("mode_system", 100, &VizLibrary::modeCb, &vizlibObj);
	//Subscribe to joint_states to record in Mode 2
	ros::Subscriber sub_joints = node_Viz.subscribe<sensor_msgs::JointState>("joint_states", 100,  &VizLibrary::jointCallback, &vizlibObj);
	
	
	//PUBLISH
	//Current vision mode
	ros::Publisher pub_v_mode = node_Viz.advertise<std_msgs::Int32>("mode_v_main",100);
	//Pixel difference publisher
	ros::Publisher pub_v_pixel = node_Viz.advertise<geometry_msgs::Twist>("pixel_difference",100);
	
	
	//Main Loop
	ros::Rate r(5);
	while(ros::ok())
	{
		switch (vizlibObj.mode_V_cmd) {
			case 0:
				//Vision is initializing and awaiting for command
				mode_Viz.data = 0;
				cout << "VIZ: Waiting for the robot..." << endl;
				break;
			case 1:
				//TBA
				break;
			case 2:
				//Run tool detection algo (with validation checks)
				mode_Viz.data = 2;
				break;
				//mode_V_cmd = 3 is never published!
			case 5:
				//Circle detection mode: Find and align to the valve
				vizlibObj.detect_circlePub();
				pub_v_pixel.publish(vizlibObj.xyzPxMsg);
				//Send current mode to SM
				mode_Viz.data = 5;
				//Check if a circle actually detected
				if ( vizlibObj.valve_Circle.empty() == 0 ) {
					//Set "Aligned" if pixel diff. between circle center and image center is less than a threshold
					if ( (abs(vizlibObj.xyzPxMsg.linear.y) <= vizlibObj.circle_Thres) && (abs(vizlibObj.xyzPxMsg.linear.z) <= vizlibObj.circle_Thres) ) {
						cout << "Valve center aligned to the camera..." << endl;
						mode_Viz.data = 6;
						//Kill the circles image
						destroyWindow("circles");
					}
				} else {
					//Valve not detected. Search for valve with small motions (at certain height)
					mode_Viz.data = 7;
				}
				break;
			case 6:
				//Tool detection mode: Detect 6 tools and
				vizlibObj.detectProcess();
				//Publish first tools pose difference from the center
				mode_Viz.data = 6;
				pub_v_pixel.publish(vizlibObj.xyzPxMsg);
				if ( (abs(vizlibObj.xyzPxMsg.linear.y) <= vizlibObj.circle_Thres) && (abs(vizlibObj.xyzPxMsg.linear.z) <= vizlibObj.circle_Thres) )
				{
					cout << "Tooltip center aligned to the camera..." << endl;
					mode_Viz.data = 7; //Send confirmation of center alignment
				}
				break;
		}
		
		//Publish the current vision mode
		pub_v_mode.publish(mode_Viz);
		//Show the current vision mode
		cout << "VIZ: mode: " << mode_Viz.data << endl;
		
		//Spin and sleep
		ros::spinOnce();
		r.sleep();
	}
	
	cv::destroyAllWindows();
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
}
