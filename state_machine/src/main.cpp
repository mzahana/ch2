#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
//OpenCV related classes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//Vision classes
#include "ModeSelect.cpp"

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
	ModeSelect modeObj ( "Idle" , "Idle" , "detectTools" , "Idle" );
	
	//Define objects
	//HuskyLibrary huskyObj;
	
	//Subscribe
	//Current modes
	//ros::Subscriber sub_H_mode = node_Sm.subscribe<std_msgs::String>("mode_h_main", 1000, &ModeSelect::mode_Husky_Cb, &odeObj);
	ros::Subscriber sub_U_mode = node_Sm.subscribe<std_msgs::String>("mode_u_main", 1000, &ModeSelect::mode_UR_Cb, &modeObj);
	ros::Subscriber sub_V_mode = node_Sm.subscribe<std_msgs::String>("mode_v_main", 1000, &ModeSelect::mode_Viz_Cb, &modeObj);
	ros::Subscriber sub_G_mode = node_Sm.subscribe<std_msgs::String>("mode_g_main", 1000, &ModeSelect::mode_Grip_Cb, &modeObj);
	//Task finished confirmations
	//ros::Subscriber sub_H_task = node_Sm.subscribe<std_msgs::Int32>("task_h_main", 1000, &ModeSelect::task_Husky_Cb, &odeObj);
	ros::Subscriber sub_U_task = node_Sm.subscribe<std_msgs::Int32>("task_u_main", 1000, &ModeSelect::task_UR_Cb, &modeObj);
	ros::Subscriber sub_V_task = node_Sm.subscribe<std_msgs::Int32>("task_v_main", 1000, &ModeSelect::task_Viz_Cb, &modeObj);
	ros::Subscriber sub_G_task = node_Sm.subscribe<std_msgs::Int32>("task_g_main", 1000, &ModeSelect::task_Grip_Cb, &modeObj);
	
	
	//Publish
	//Commanded modes
	//ros::Publisher pub_modeHusky = node_Sm.advertise<std_msgs::String>("cmdmode_husky", 1000);
	ros::Publisher pub_modeUr = node_Sm.advertise<std_msgs::String>("cmdmode_ur", 1000);
	ros::Publisher pub_modeViz = node_Sm.advertise<std_msgs::String>("cmdmode_viz", 1000);
	ros::Publisher pub_modeGrip = node_Sm.advertise<std_msgs::String>("cmdmode_grip", 1000);
    

	ros::Rate r(20);
	while(ros::ok())
	{
		//Run the modeSwitch fcn if there is no current active job
		if ( (modeObj.finished_H == 1) && (modeObj.finished_U == 1) && (modeObj.finished_V == 1) && (modeObj.finished_G == 1) ) {
			modeObj.modeSwitch();
		}
		
		//Publish the commanded system mode
		//pub_modeHusky.publish(modeObj.M_h);
		pub_modeUr.publish(modeObj.M_u);
		pub_modeViz.publish(modeObj.M_v);
		pub_modeGrip.publish(modeObj.M_g);

		//Show system state
		cout << "MAIN: States: H[" << modeObj.mode_Husky << "], U[" << modeObj.mode_UR << "], V[" << modeObj.mode_Viz << "], G[" << modeObj.mode_Grip_SM << "]" << endl;
		cout << "MAIN: Commands: H[" << modeObj.M_h << "] - U[" << modeObj.M_u << "] - V[" << modeObj.M_v << "] - G[" << modeObj.M_g << "]" << endl;
		cout << endl;

		//Spin and sleep
		ros::spinOnce();
		r.sleep();
	}
	
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
	return 0;
}
