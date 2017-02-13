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
#include "CascadeDet.cpp"
#include "ValveDet.cpp"
//#include "IdentifySize.cpp"


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
	ModeSelect modeObj;
	std_msgs::Int16MultiArray mode_State;

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
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, &CascadeDet::imageCallback, &cascadeObj);
	//System mode
	ros::Subscriber sub_mode_system = node_Viz.subscribe<std_msgs::Int32MultiArray>("mode_system", 100, modeCb);
	//Subscribe to joint_states to record in Mode 2
	ros::Subscriber sub_joints = node_Viz.subscribe<sensor_msgs::JointState>("joint_states", 100, jointCallback);
	
	
	//PUBLISH
	//Current vision mode
	ros::Publisher pub_v_mode = node_Viz.advertise<std_msgs::Int32>("mode_v_main",100);
	//Center of six tools detected (used in Mode 2)
	ros::Publisher pub_v_center = node_Viz.advertise<geometry_msgs::Twist>("viz_tool_center",100);
	
	
	//Main Loop
	ros::Rate r(5);
	while(ros::ok())
	{
		switch (mode_V_cmd) {
			case 0:
				//Vision is initializing and awaiting for command
				cout << "VIZ: Waiting for the robot..." << endl;
			case 1:
				//TBA
			case 2:
				//Run tool detection algo (with validation checks)
				cascadeObj.detectProcess();
				//Publish center of 6 tools
				pub_v_center.publish(tool_center_diff);
				if ( (tool_center_diff.linear.x <= cen_thres_x) && (tool_center_diff.linear.y <= cen_thres_y) )
				{
					mode_Viz = 3; //Send confirmation of center alignment
				}
				//mode_V_cmd = 3 is never published!
			case 4:
				//Tracking mode: Track the leftmost (first) tool
				//trackObj.();
				
				
			default:
				//Vision is initializing and awaiting for command
				cout << "VIZ: No command received!" << endl;
		}
		
		//Publish the current vision mode
		pub_v_mode.publish(mode_Viz);
		//Show the current vision mode
		cout << "VIZ: mode: " << mode_Viz << endl;
		
		//Spin and sleep
		ros::spinOnce();
		r.sleep();
	}
	
	cv::destroyAllWindows();
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
}
