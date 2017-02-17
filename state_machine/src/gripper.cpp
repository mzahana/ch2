#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
//OpenCV related classes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//Required custom libraries


using namespace std;
using namespace cv;


//Public vars
string mode_G_cmd;
int grip_ActMode (0);
vector<double> grip_Imu;
int grip_pinch_no (0);


//Gripper actual mode callback
void gripActualCb(const std_msgs::Int32::ConstPtr& msgActGrip)
{
	grip_ActMode = msgActGrip -> data;
}

//Gripper command callback (from SM)
void gripCmdCb(const std_msgs::String::ConstPtr& msgCmdGrip)
{
	mode_G_cmd = msgCmdGrip -> data;
}

//Gripper command callback
void gripImuCb(const geometry_msgs::Vector3::ConstPtr& msgImuGrip)
{
	grip_Imu[0] = msgImuGrip -> x;
	grip_Imu[1] = msgImuGrip -> y;
	grip_Imu[2] = msgImuGrip -> z;
}

//Gripper pinch number callback
void gripPinchCb(const std_msgs::Int32::ConstPtr& msgPinchGrip)
{
	grip_pinch_no = msgPinchGrip -> data;
}





/***********************/
//Gripper interface function. Provides communication between Arduino and SM
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_node");
	ros::NodeHandle node_Grip;
	
	
	//Variables published
	std_msgs::String mode_Grip; //Current gripper mode published to SM
	std_msgs::Int32 finished_G; //Task finish confirmation published to SM
	std_msgs::Int32 grip_cmd_ard; //Grip commands sent to Arduino

	finished_G.data = 1; //Start with confirmation
	
	//SUBSCRIBE
	//Gripper "actual" mode (coming from h/w)
	ros::Subscriber sub_actualMode = node_Grip.subscribe<std_msgs::Int32>("mode_g_main", 100, gripActualCb);
	//Gripper command mode (coming from SM)
	ros::Subscriber sub_cmd_grip = node_Grip.subscribe<std_msgs::String>("cmdmode_grip", 100, gripCmdCb);
	//Gripper IMU (coming from h/w)
	ros::Subscriber sub_imu_grip = node_Grip.subscribe<geometry_msgs::Vector3>("imu_gripper", 100, gripImuCb);
	//Gripper pinch command (coming from h/w)
	ros::Subscriber sub_pinch_fb = node_Grip.subscribe<std_msgs::Int32>("pinch_feedback", 100, gripPinchCb);
	
	
	//PUBLISH
	//Current gripper mode published to SM
	ros::Publisher pub_v_mode = node_Grip.advertise<std_msgs::Int32>("mode_system_grip",100);
	//Pinch number
	ros::Publisher pub_pinch = node_Grip.advertise<std_msgs::Int32>("pinch",100);
	//Finished task confirmation published to SM
	ros::Publisher pub_taskFinish = node_Grip.advertise<std_msgs::Int32>("task_g_main",100);
	
	
	//Main Loop
	ros::Rate r(10);
	while(ros::ok())
	{
		if (mode_G_cmd == "Idle") {
			//Gripper is supposed to await for command
			grip_cmd_ard.data = 0; //Sends the Idle command to Arduino
			if (grip_ActMode == 0) {
				//Check if gripper is really in Idle
				mode_Grip.data = "Idle";
				cout << "GRIP: Waiting for command..." << endl;
			} else {
				cout << "GRIP: Gripper is not in Idle!!!" << endl;
			}
			finished_G.data = 1; //Ready to get task
		} else if (mode_G_cmd == "pinch") {
			//Gripper pinch TBA
			mode_Grip.data = "pinching";
			cout << "GRIP: Gripper performing pinching..." << endl;
		} else if (mode_G_cmd == "gripTool") {
			//Grip the tool
			mode_Grip.data = "gripping";
			//Perform gripping by sending command to Arduino
			grip_cmd_ard.data = 2; //Sends the grip command to Arduino
		} else if (grip_ActMode == 3) {
			//Gripper is closing, we are waiting for gripper confirmation
			mode_Grip.data = "Gripping";
			cout << "GRIP: Gripper is closing..." << endl;
		} else if (grip_ActMode == 4) {
			//Gripper is closed, inform SM
			mode_Grip.data = "gripped";
			//Tell SM that vision finished the task
			finished_G.data = 1;
			cout << "GRIP: Gripper closed..." << endl;
		}
		
		//Publish task finish confirmation
		pub_taskFinish.publish(finished_G);
		//Publish the current vision mode
		pub_v_mode.publish(mode_Grip);
		//Show the current vision mode
		cout << "GRIP: mode: " << mode_Grip.data << endl;
		
		//Spin and sleep
		ros::spinOnce();
		r.sleep();
	}
	
	cv::destroyAllWindows();
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
}
