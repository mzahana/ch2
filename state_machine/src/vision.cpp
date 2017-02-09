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



//Vision main function. Manages CascadeDet, ValveDet, and Identify classes
//Subscribes to M_v, publishes mode_Viz
int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_main");
	ros::NodeHandle node_Viz;

	//Mode select class object
	//Manages current mode based on state/information flow
	ModeSelect modeObj;
	std_msgs::Int16MultiArray mode_State;
	
	//Cascade detection object
	CascadeDet cascadeObj;
	
	//Hough circles object
	ValveDet valveObj;
	
	//Size identification object
	IdentifySize identObj;
	
	//Subscribe
	ros::Subscriber sub_mode_system = node_Viz.subscribe<std_msgs::Int16MultiArray>("mode_system", 1000);
	
    //Publish
	ros::Publisher pub_v_mode = node_Viz.publish<>
	

	ros::Rate r(10);
	while(ros::ok())
	{
		//Get current system mode
		mode_State.clear(); //Clear array before filling
		mode_State = modeObj.modeSwitch(modeObj.mode_Husky, modeObj.mode_UR, modeObj.mode_Viz, modeObj.mode_Grip);
		
		//Publish the commanded system mode
		pub_mode.publish(mode_State);

		//Show system state
        cout << "MAIN: mode: " << mode_system << endl;
		
		//Spin and sleep
        ros::spinOnce();
		r.sleep();
	}

	cv::destroyAllWindows();
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
}
