#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace cv;


class ModeSelect
{
public:
	//Global variables
	int modeNo;

	ModeSelect(){
		modeNo = 0;
	};

	int modeSwitch(int radiusValve){
		modeNo = 0;
		if (radiusValve >= 10){modeNo = 1;}
		return modeNo;
	}
	
};