#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv.h"
#include "ml.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <boost/lexical_cast.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

using namespace std;
using namespace cv;
using namespace Eigen;


//Object tracking class
//Input: Detected tooltip box (type: rectangle)
//Output: Integer representing the result
class TrackTool
{
public:
	//Global vars
	
	//Constructor
	TrackTool(){
	}
	
	//Object tracking function
	int trackTip(Mat frame, vector<Rect> tool_Tip){
	}
	
	
	
	
private:
	//Tracks the object detected and stored in the tool_Tip rectangle
	int trackTip(Mat frame, vector<Rect> tool_Tip){
		
		//Return the current i_Level for checking stopping time of this function
		return 0;
	}
		
};
