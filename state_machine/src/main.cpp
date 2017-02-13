#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/int32.h"
#include "std_msgs/Int32MultiArray.h"
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
	//Initialize with desired system state
	ModeSelect modeObj ( 0 , 0 , 0 , 0 );
	std_msgs::Int32MultiArray mode_State;
	
	//Define objects
	HuskyLibrary huskyObj;
	UrLibrary urObj;
	VizLibrary visionObj;
	GripLibrary gripperObj;
	
	//Subscribe
	ros::Subscriber sub_H_mode = node_Sm.subscribe<std_msgs::Int32>("mode_h_main", 1000, &ModeSelect::mode_Husky_Cb, &huskyObj);
	ros::Subscriber sub_U_mode = node_Sm.subscribe<std_msgs::Int32>("mode_u_main", 1000, &ModeSelect::mode_UR_Cb, &urObj);
	ros::Subscriber sub_V_mode = node_Sm.subscribe<std_msgs::Int32>("mode_v_main", 1000, &ModeSelect::mode_Viz_Cb, &visionObj);
	ros::Subscriber sub_G_mode = node_Sm.subscribe<std_msgs::Int32>("mode_g_main", 1000, &ModeSelect::mode_Grip_Cb, &gripperObj);
	
	
    //Publish
	ros::Publisher pub_mode = node_Sm.advertise<std_msgs::Int32MultiArray>("mode_system", 1000);
    

	ros::Rate r(20);
	while(ros::ok())
	{
		//Get current system mode
		mode_State.clear(); //Clear array before filling
		mode_State[0] = modeObj.mode_Husky;
		mode_State[1] = modeObj.mode_UR;
		mode_State[2] = modeObj.mode_Viz;
		mode_State[3] = modeObj.mode_Grip;
		
		//Publish the commanded system mode
		mode_Cmd = modeObj.modeSwitch(modeObj.mode_Husky, modeObj.mode_UR, modeObj.mode_Viz, modeObj.mode_Grip);
		pub_mode.publish(modeObj.mode_Cmd);

		//Show system state
        cout << "MAIN: System state: H[" << mode_State[0] << "], U[" << mode_State[1] << "], V[" << mode_State[2] << "], G[" << mode_State[3] << "]" << endl;
		
		//Spin and sleep
        ros::spinOnce();
		r.sleep();
	}

	cv::destroyAllWindows();
	//Delete the NN pointer
	delete visionObj.nnetwork;
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
}
