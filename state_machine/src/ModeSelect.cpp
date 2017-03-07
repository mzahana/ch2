#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

using namespace std;
using namespace cv;




class ModeSelect
{
public:
	String mode_UR, mode_Viz, mode_Grip_SM; //Subscribed modes
	std_msgs::String M_h, M_u, M_v, M_g; //Published modes (Default vals)
	int mode_Husky, finished_H, finished_U, finished_V, finished_G; //Active task confirmation
	int state_Husky, state_Count, count_goCorrectTool, count_atTools;

	//Constructor
	ModeSelect(int m0, string m1, string m2, string m3){
		//Initialize all mode vars
		mode_Husky = m0; mode_UR = m1; mode_Viz = m2; mode_Grip_SM = m3;
		M_h.data = "Idle"; M_u.data = "Idle"; M_v.data = "Idle"; M_g.data = "Idle";
		finished_H = 1; finished_U = 1; finished_V = 1; finished_G = 1;
		state_Husky = 0; //Not used
		state_Count = 0; //Not used
		count_goCorrectTool = 0;
		count_atTools = 0;
	};
	
	
	//Husky current mode callback
	void mode_Husky_Cb(const std_msgs::Int32::ConstPtr& mode_Hcur){
		mode_Husky = mode_Hcur -> data;
		//cout << "Mode Husky: " << mode_Husky << endl;
	}
	
	//UR5 current mode callback
	void mode_UR_Cb(const std_msgs::String::ConstPtr& mode_URcur){
		mode_UR = mode_URcur -> data;
		//cout << "Mode UR: " << mode_UR << endl;
	}
	//Vision current mode callback
	void mode_Viz_Cb(const std_msgs::String::ConstPtr& mode_Viscur){
		mode_Viz = mode_Viscur -> data;
		//cout << "Mode Vision: " << mode_Viz << endl;
	}
	//Gripper current mode callback
	void mode_Grip_Cb(const std_msgs::String::ConstPtr& mode_Gripcur){
		mode_Grip_SM = mode_Gripcur -> data;
		//cout << "Mode Grip: " << mode_Grip << endl;
	}
	
	/********************/
	/*
	//Husky task finished callback - Not used
	void task_Husky_Cb(const std_msgs::Int32& task_Husky){
		finished_H = task_Husky -> data;
		//cout << "Finished Husky: " << finished_H << endl;
	}
	*/
	//UR5 task finished callback
	void task_UR_Cb(const std_msgs::Int32::ConstPtr& task_UR){
		finished_U = task_UR -> data;
		//cout << "Finished UR: " << finished_U << endl;
	}
	//Vision task finished callback
	void task_Viz_Cb(const std_msgs::Int32::ConstPtr& task_Viz){
		finished_V = task_Viz -> data;
		//cout << "Finished Vision: " << finished_V << endl;
	}
	//Gripper task finished callback
	void task_Grip_Cb(const std_msgs::Int32::ConstPtr& task_Grip){
		finished_G = task_Grip -> data;
		//cout << "Finished Grip: " << finished_G << endl;
	}
	
	
	/********************/
	//Main mode selection function
	int modeSwitch(){
		//Until Husky-Park mode (=10) finishes, deactivate the rest
		if ( mode_Husky != 10 ){
			M_u.data = "Idle"; M_v.data = "Idle"; M_g.data = "Idle";
		}
		
		//After Husky aligned, get UR to ready pose
		if ( mode_Husky == 10 ){
			if ( (mode_UR == "Idle") && (state_Count == 0) ) {
				M_u.data = "Ready"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( mode_UR == "urAligned" ) {
				//(Internally switched). Means UR aligned to the panel and came back,
				//and elevated to find valve circle
				//Switch to valve search mode
				M_u.data = "searchValve"; M_v.data = "valveViz"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( mode_Viz == "valveFound" ) {
				//(Internally switched). Means the valve circle found by vision
				//Switch to valve align mode
				M_u.data = "valveAlign"; M_v.data = "valveAlign"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( (mode_UR == "valveAlign") && (mode_Viz == "valveAligned") ) {
				//mode_Viz Internally switched: means camera is aligned to valve up to threshold of 1 pixel
				//Move UR to the cup-valve position
				M_u.data = "valveSizing"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( (mode_UR == "sizingDone") && (mode_Viz != "sizingDone") ) {
				//mode_UR Internally switched: UR is at cup-valve pose
				//Run getValvesize.py to get the valve size
				M_u.data = "Idle"; M_v.data = "valveSizing"; M_g.data = "Idle";
				finished_V = 0;
			}
			else if ( mode_Viz == "sizingDone" ) {
				//Valve size is detected, go to pins
				//Move to the valve axis
				M_u.data = "goPins"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( (mode_UR == "atPins") && (mode_Viz != "pinsAligned") ) {
				//UR is at pins. Run vision alignPins mode
				M_u.data = "Idle"; M_v.data = "alignPins"; M_g.data = "Idle";
				finished_V = 0;
			}
			else if ( (mode_UR == "atPins") && (mode_Viz == "pinsAligned") ) {
				//Pins are aligned. Move down to tools
				M_u.data = "goTools"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( (mode_UR == "atTools") && (mode_Viz != "toolsSized") && (count_atTools == 0) ) {
				//UR is at tools. Run tool detection and move UR a bit
				M_u.data = "scanTools"; M_v.data = "detectTools"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( (mode_Viz == "toolsSized") && (M_u.data != "goCorrectTool") && (count_goCorrectTool == 0) ) {
				//Tool sizes detected. Go to the correct tool height
				M_u.data = "goCorrectTool"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
				count_goCorrectTool = 1; //Run this mode once
				count_atTools = 1;
			}
			else if ( mode_UR == "atCorrectTool" ) {
				//mode_UR internally switched: UR is at the correct tool height at the back plane
				//Align UR with the correct tool "y" - This mode directly switches to the next, NOT used
				M_u.data = "alignCorrectTool"; M_v.data = "alignCorrectTool"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( (mode_UR == "alignCorrectTool") && (mode_Viz == "correctAligned") ) {
				//Tool alignment finished. Approach the correct tool for gripping
				M_u.data = "goGripping"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( (mode_UR == "atGrip" ) && (M_g.data != "gripTool")) {
				//UR is at the gripping position. Run gripper
				M_u.data = "Idle"; M_v.data = "Idle"; M_g.data = "gripTool";
				finished_G = 0;
			}
			else if ( (mode_Grip_SM == "gripped") && (M_u.data != "moveValve") && (mode_UR != "atValve") ) {
				//Gripping completed. Move back to valve
				//Keep sending gripper command to avoid false opening
				M_u.data = "moveValve"; M_v.data = "Idle"; M_g.data = "gripTool";
				finished_U = 0;
			}
			else if ( (mode_UR == "atValve") ) {
				//UR is at valve. Run rotation
				//Keep sending gripper command to avoid false opening
				M_u.data = "rotateValve"; M_v.data = "Idle"; M_g.data = "gripTool";
				finished_U = 0;
			}
		}
		return 0;
	}
};
