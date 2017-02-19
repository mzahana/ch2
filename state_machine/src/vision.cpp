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
	vizlibObj.detectAndIdent();
	
	
	//Variables published
	std_msgs::String mode_Viz;
	std_msgs::Int32 finished_V;
	
	//SUBSCRIBE
	//Image transport object
	image_transport::ImageTransport it(node_Viz);
	image_transport::Subscriber sub = it.subscribe("usb_cam/image_rect_color", 1, &VizLibrary::imageCallback, &vizlibObj);
	//System mode
	ros::Subscriber sub_mode_system = node_Viz.subscribe<std_msgs::String>("cmdmode_viz", 100, &VizLibrary::modeCb, &vizlibObj);
	//Subscribe to joint_states to record in Mode 2
	ros::Subscriber sub_joints = node_Viz.subscribe<sensor_msgs::JointState>("joint_states", 100,  &VizLibrary::jointCallback, &vizlibObj);
	
	
	//PUBLISH
	//Current vision mode
	ros::Publisher pub_v_mode = node_Viz.advertise<std_msgs::String>("mode_v_main",100);
	//Task finish confirmation mode
	ros::Publisher pub_v_fin = node_Viz.advertise<std_msgs::Int32>("task_v_main",100);
	//Pixel difference publisher
	ros::Publisher pub_v_pixel = node_Viz.advertise<geometry_msgs::Twist>("pixel_difference",100);
	
	
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
		} else if (vizlibObj.mode_V_cmd == "detectTools") {
			//Tool detection mode: Detect 6 tools and if found return
			if ( vizlibObj.level_Count <= vizlibObj.n_Level-1 ) {
				int detect_Res = vizlibObj.detectAndIdent();
				mode_Viz.data = "detectingTools";
			} else {
				mode_Viz.data = "toolDetectionFinished";
			}
		} else if (vizlibObj.mode_V_cmd == "alignCorrectTool") {
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
		}
		
		
		//Tell SM that vision finished the task
		finished_V.data = 1;
		pub_v_fin.publish(finished_V);
		//Publish the current vision mode
		pub_v_mode.publish(mode_Viz);
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
