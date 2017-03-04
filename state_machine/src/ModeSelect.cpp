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
		state_Husky = 0;
		state_Count = 0;
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
	//Husky task finished callback
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
		//Until Husky-Align mode finishes, deactivate the rest
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
				//and elevated to find tools ROI.
				//Switch to tool detection mode
				M_u.data = "searchValve"; M_v.data = "valveViz"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( mode_Viz == "valveFound" ) {
				//(Internally switched). Means UR aligned to the panel and came back,
				//and elevated to find tools ROI.
				//Switch to tool detection mode
				M_u.data = "valveAlign"; M_v.data = "valveAlign"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( (mode_UR == "valveAlign") && (mode_Viz == "valveAligned") ) {
				//mode_Viz Internally switched: means 6 tool tips are detected
				//Run tracking of the first detected tool and align UR with that
				M_u.data = "valveSizing"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( (mode_UR == "sizingDone") && (mode_Viz != "sizingDone") ) {
                                //mode_Viz Internally switched: means 6 tool tips are detected
                                //Run tracking of the first detected tool and align UR with that
                                M_u.data = "Idle"; M_v.data = "valveSizing"; M_g.data = "Idle";
                                finished_V = 0;
                        }
			else if ( mode_Viz == "sizingDone" ) {
				//UR is aligned with the first tool axis.
				//Move to the valve axis
				M_u.data = "goPins"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( (mode_UR == "atPins") && (mode_Viz != "pinsAligned") ) {
                                //UR is aligned with the first tool axis.
                                //Move to the valve axis
                                M_u.data = "Idle"; M_v.data = "alignPins"; M_g.data = "Idle";
				finished_V = 0;
                        }
			else if ( (mode_UR == "atPins") && (mode_Viz == "pinsAligned") ) {
                                //mode_Husky internally switched: UR is on valve location
                                //Run valve detection and align with the valve
                                M_u.data = "goTools"; M_v.data = "Idle"; M_g.data = "Idle";
                                finished_U = 0;
                        }
			else if ( (mode_UR == "atTools") && (mode_Viz != "toolsSized") && (count_atTools == 0) ) {
				//mode_Husky internally switched: UR is on valve location
				//Run valve detection and align with the valve
				M_u.data = "scanTools"; M_v.data = "detectTools"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( (mode_Viz == "toolsSized") && (M_u.data != "goCorrectTool") && (count_goCorrectTool == 0) ) {
				M_u.data = "goCorrectTool"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
				count_goCorrectTool = 1;
				count_atTools = 1;
			}
			else if ( mode_UR == "atCorrectTool" ) {
				//mode_UR internally switched: UR approached to valve for pinching
				//Run gripper pinch, and rotate Husky for different pinch angles
				M_u.data = "alignCorrectTool"; M_v.data = "alignCorrectTool"; M_g.data = "Idle";
				finished_U = 0;
				finished_V = 0;
			}
			else if ( (mode_UR == "alignCorrectTool") && (mode_Viz == "correctAligned") ) {
				//Pinch finished: Valve size detected. Move back to tools
				M_u.data = "goGripping"; M_v.data = "Idle"; M_g.data = "Idle";
				finished_U = 0;
			}
			else if ( (mode_UR == "atGrip" ) && (M_g.data != "gripTool")) {
				//Pinch operation: Increment a counter in gripper so that it is operated finite times
				//Valve size detected. Move back to tools
				M_u.data = "Idle"; M_v.data = "Idle"; M_g.data = "gripTool";
				finished_G = 0;
			}
			else if ( (mode_Grip_SM == "gripped") && (M_u.data != "moveValve") && (mode_UR != "atValve") ) {
				//Pinch completed. Move back
				M_u.data = "moveValve"; M_v.data = "Idle"; M_g.data = "gripTool";
				finished_U = 0;
			}
			else if ( (mode_UR == "atValve") ) {
				//Move to first tool
				M_u.data = "rotateValve"; M_v.data = "Idle"; M_g.data = "gripTool";
				finished_U = 0;
			}
			else {
				M_u.data = "Idle"; M_v.data = "Idle"; M_g.data = "Idle";
			}
		}
		return 0;
	}
};
