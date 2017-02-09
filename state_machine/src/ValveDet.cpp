#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/image_encodings.h>


using namespace std;
using namespace cv;


//Class for valve circle and center detection.
//Input: image frame and frame_gray
//Output: Circle center and radius
class ValveDet
{
public:
	Mat frame;
	geometry_msgs::Twist xyzPxMsg;
	int radiusValve;

	void detect_valve(Mat frame, Mat gray){
		if (frame.empty() == 0) {
			Mat img = frame;
		    GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
		    vector<Vec3f> circles;
		    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, 480/4, 200, 100 );

		    for( size_t i = 0; i < circles.size(); i++ )
		    {
				Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				radiusValve = cvRound(circles[i][2]);
				// draw the circle center
				circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
				// draw the circle outline
				circle( img, center, radiusValve, Scalar(0,0,255), 3, 8, 0 );

				if (i == 0) {
					xyzPxMsg.linear.x = center.x - ceil(img.cols/2);
					xyzPxMsg.linear.y = center.y - ceil(img.rows/2);
				}
		    }
		    
		    imshow( "circles", img );
		    //imshow("grayscale",gray);
		    waitKey(1);
		}
	}
};
