#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point32.h"
//OpenCV related classes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"


//Vision classes
#include "VizLibrary.cpp"


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
	//vizlibObj.trainNN();
	//vizlibObj.detectAndIdent();
	
	
	//Variables published
	std_msgs::String mode_Viz;
	std_msgs::Int32 finished_V;
	std_msgs::Int32 pin_Number;
	std_msgs::Int32 valve_Size;
	std_msgs::Int32 valve_Angle;
	std_msgs::Float64 cam_offset;
	std_msgs::String tool_Face;

	int valve_Size_Detected = 0;

	
	//SUBSCRIBE
	//Image transport object
	image_transport::ImageTransport it(node_Viz);
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_rect_color", 1, &VizLibrary::imageCallback, &vizlibObj);
	//System mode
	ros::Subscriber sub_mode_system = node_Viz.subscribe<std_msgs::String>("cmdmode_viz", 100, &VizLibrary::modeCb, &vizlibObj);
	//Subscribe to joint_states to record in Mode 2
	ros::Subscriber sub_joints = node_Viz.subscribe<sensor_msgs::JointState>("joint_states", 100,  &VizLibrary::jointCallback, &vizlibObj);
	//System mode
	//ros::Subscriber sub_valve = node_Viz.subscribe<geometry_msgs::Point32>("valveStats", 100, &VizLibrary::modeconfirmCb, &vizlibObj);
	//Subscribe to joint_states to record in Mode 2
	ros::Subscriber sub_Size = node_Viz.subscribe<std_msgs::Int32>("valve_size", 100,  &VizLibrary::valveSizeCb, &vizlibObj);
	//System mode
	ros::Subscriber sub_Angle = node_Viz.subscribe<std_msgs::Int32>("valve_angle", 100, &VizLibrary::valveAngleCb, &vizlibObj);
	
	
	//PUBLISH
	//Current vision mode
	ros::Publisher pub_v_mode = node_Viz.advertise<std_msgs::String>("mode_v_main",100);
	//Task finish confirmation mode
	ros::Publisher pub_v_fin = node_Viz.advertise<std_msgs::Int32>("task_v_main",100);
	//Pixel difference publisher
	ros::Publisher pub_v_pixel = node_Viz.advertise<geometry_msgs::Twist>("pixel_difference",100);
	//Pin number publisher
	ros::Publisher pub_v_pin = node_Viz.advertise<std_msgs::Int32>("pin_number",100);
	//Valve size publisher
	ros::Publisher pub_v_size = node_Viz.advertise<std_msgs::Int32>("valve_sizeUr",100);
	//Valve size publisher
	ros::Publisher pub_v_angle = node_Viz.advertise<std_msgs::Int32>("valve_angleUr",100);
	//Camera-pin offset publisher
	ros::Publisher pub_v_camoffset = node_Viz.advertise<std_msgs::Float64>("cam_offset",100);
	//Tool face publisher
	ros::Publisher pub_v_toolface = node_Viz.advertise<std_msgs::String>("tool_face",100);
	
	
	//Main Loop
	ros::Rate r(2);
	while(ros::ok())
	{
		if (vizlibObj.mode_V_cmd == "Idle") {
			//Vision is initializing and awaiting for command
			mode_Viz.data = "Idle";
			cout << "VIZ: Waiting for the robot..." << endl;
		} else if (vizlibObj.mode_V_cmd == "valveViz") {
			//Run valve detection algo, if circle found return
			vizlibObj.detect_circlePub();
			cout << "no of circles: " << vizlibObj.circles.size() << endl;
			if ( vizlibObj.circles.empty() == 0 ) {
				mode_Viz.data = "valveFound";
			} else {
				mode_Viz.data = "valveViz";
			}
		} else if (vizlibObj.mode_V_cmd == "valveAlign") {
			//Run valve detection algo, if circle found return
			vizlibObj.detect_circlePub();
			//Align the camera to the valve center
			if ( vizlibObj.circles.empty() == 0 ) {
				//Set "Aligned" if pixel diff. between circle center and image center is less than a threshold
				if ( (abs(vizlibObj.xyzPxMsg.linear.y) <= vizlibObj.pixelThres) && (abs(vizlibObj.xyzPxMsg.linear.z) <= vizlibObj.pixelThres) ) {
					cout << "Valve center aligned to the camera..." << endl;
					//Update the current mode
					mode_Viz.data = "valveAligned";
					//Kill the circles image
					destroyWindow("circles");
				}
			} else {
				//Valve not detected. Search for valve with small motions (at certain height)
				mode_Viz.data = "aligningValve";
			}
			pub_v_pixel.publish(vizlibObj.xyzPxMsg);
		} else if (vizlibObj.mode_V_cmd == "valveSizing") {
			//Valve sizing
			sleep(10);
			for (int i = 0; i < 20; ++i)
			{
				//Valve size publisher
				valve_Size.data = vizlibObj.valveSize;
				pub_v_size.publish(valve_Size);
				//Valve angle publisher
				valve_Angle.data = vizlibObj.valveAngle;
				pub_v_angle.publish(valve_Angle);
				sleep(0.1);
			}
			vizlibObj.valve_Size_Detected = valve_Size.data;
			cout << "Valve size: " << valve_Size.data << endl;
			cout << "Valve angle: " << valve_Angle.data << endl;
			cout << "Valve size passed: " << vizlibObj.valve_Size_Detected << endl;
			sleep(2);
			//Update the current mode
			mode_Viz.data = "sizingDone";
		} else if (vizlibObj.mode_V_cmd == "alignPins") {
			//Valve sizing
			vizlibObj.detect_pinsPub();
			cam_offset.data = vizlibObj.orient_Offset;
			//Update the current mode
			if ( vizlibObj.circles.empty() == 0 ) {
				//Set "Aligned" if pixel diff. between circle center and image center is less than a threshold
				if ( abs(vizlibObj.orient_Offset) <= 1 ) {
					cout << "Pins aligned to the camera..." << endl;
					//Update the current mode
					mode_Viz.data = "pinsAligned";
					//Kill the circles image
					//destroyWindow("circles_pins");
				}
			} else {
				//Valve not detected. Search for valve with small motions (at certain height)
				mode_Viz.data = "aligningPins";
			}
			//Camera-pin offset publisher
			pub_v_camoffset.publish(cam_offset);
		} else if ((mode_Viz.data != "toolsSized") && (vizlibObj.mode_V_cmd == "detectTools")) {
			//Tool detection mode: Detect 6 tools and if found return
			int detect_Res = vizlibObj.detectAndIdent();
			//If executed n_Level detection, publish the pin_number
			if ( (vizlibObj.checkDetection == 0) && (vizlibObj.level_Count <= vizlibObj.n_Level - 1 )) {
				//Update the current mode
				mode_Viz.data = "detectingTools";
			} else {
				pin_Number.data = vizlibObj.pin_Number_Detected;
				//Pin number publisher
				for (int i = 0; i < 50; ++i)
				{
					pub_v_pin.publish(pin_Number);
					sleep(0.05);
				}
				//Update the current mode
				mode_Viz.data = "toolsSized";
				cout << "FINISHED!!!!!!!!!!" << endl;
				cout << endl;
			}
		} else if (vizlibObj.mode_V_cmd == "alignCorrectTool") {
			/*
			//Arm approached the correct tool, align the camera with the tool
			int detect_Res = vizlibObj.detectProcess();
			//If the tool is detected, send the pixel differences to the arm
			if ( vizlibObj.tools.empty() == 0 ) {
				//Set "Aligned" if pixel diff. between circle center and image center is less than a threshold
				if ( (abs(vizlibObj.xyzPxMsg.linear.y) <= vizlibObj.pixelThres) && (abs(vizlibObj.xyzPxMsg.linear.z) <= vizlibObj.pixelThres) ) {
					cout << "Tool center aligned to the camera..." << endl;
					//Update the current mode
					mode_Viz.data = "correctAligned";
				}
			}
			*/
			mode_Viz.data = "correctAligned";
		}
		
		
		//Tell SM that vision finished the task
		finished_V.data = 1;
		pub_v_fin.publish(finished_V);
		//Publish the current vision mode
		pub_v_mode.publish(mode_Viz);
		//Publish the tool face detected
		tool_Face.data = "Left";
		pub_v_toolface.publish(tool_Face);
		//Show the current vision mode
		cout << "VIZ: cmd: " << vizlibObj.mode_V_cmd << endl;
		cout << "VIZ: mode: " << mode_Viz.data << endl;
		
		//Spin and sleep
		ros::spinOnce();
		r.sleep();
	}
	
	cv::destroyAllWindows();
	ROS_INFO("State_machine::main.cpp::Finished with no error.");
}
