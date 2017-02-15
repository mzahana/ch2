#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv.h"
#include "ml.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <boost/lexical_cast.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <sensor_msgs/JointState.h>


using namespace std;
using namespace cv;

class VizLibrary
{
public:
	//Global vars
	String cascade_name, video_name, window_name, data_mean, pceNN, nnInput_Data, nn_Output;
	CascadeClassifier casClassifier;
	vector<Rect> tools, _lasttools, tools_Overlap;
	vector<Rect> tools_Ordered; //Vector to store ascending ordered tools (order of initial x)
	double rect_Overlap_rat;
	int cascade_Count;
	int mode_V_cmd;
	//Joint pose
	vector<double> joint_Pos;
	
	//Neural net vars
	CvANN_MLP* nnetwork;
	Mat layers;
	int min_L, min_W, no_of_in_layers, no_of_hid_layers, no_of_out_layers;
	vector<double> data_mean_vec, nn_Output_vec;
	
	//Frame matrices
	Mat frame, frame_gray, frame_to_ident, frame_color_ident;
	Mat pce_mat, nnInput_Data_mat, nnInput_Data_matTr, nnTrain_out;
	
	//Separation test vars
	vector<int> tools_Sep;
	int sep_Pixel_min, sep_Pixel_max;
	
	//Circle detection vars
	circle_Thres = 2;
	
	
	//Publish vars (automatically initializes with zero vals)
	geometry_msgs::Twist xyzPxMsg;

	

	//Constructor
	VizLibrary(){
		//Load the cascade classifier
		cascade_name = "/home/risc/catkin_ws/src/smachine/src/rexmlfiles/toolDetector6_1(1).xml";
		if( !casClassifier.load( cascade_name ) ){ printf("--(!)Error loading xml file\n"); };
		//Parameter initializations
		min_L = 36, min_W = 36;
		no_of_in_layers = 27; no_of_hid_layers = 27; no_of_out_layers = 1;
		layers = (Mat_<int>(1,3) << no_of_in_layers,no_of_hid_layers,no_of_out_layers);
		rect_Overlap_rat = 0.3;
		for (int i = 0; i < joint_Pos.size(); i++){ joint_Pos[i] = 0; }
		//Separation check vars: Set based on distance from panel
		sep_Pixel_min = 40;
		sep_Pixel_max = 150;
		//Cascade detection number of successful 6-tool-validation
		cascade_Count = 0;
		//Mode vars
		//mode_V_cmd = 0;
	}
	
	
	//Vision mode callback
	void modeCb(const std_msgs::Int32::ConstPtr& msgMode)
	{
		int mode_V_cmd = msgMode -> data;
	}
	
	
	//Encoder Callback
	void jointCallback(const sensor_msgs::JointState::ConstPtr& msgJoint)
	{
		const vector<string> joint_Names = msgJoint -> name;
		if (joint_Names[0] == "shoulder_pan_joint") {
			joint_Pos = msgJoint -> position;
		}
	}
	
	
	//Image callback
	void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("detectionOpenCV::main.cpp::cv_bridge exception: %s", e.what());
			return;
		}
		frame = cv_ptr -> image;
		cvtColor( cv_ptr -> image, frame_gray, COLOR_BGR2GRAY );
	}

	
	//Cascade detection function
	int detectProcess(){
		cout << "VIZ: Frame reading started..." << endl;
		if ( frame.empty() == 0 ) {
			tools_Ordered = detectAndDisplay( frame );
			//Apply separation test
			if( !separationTest(tools_Ordered) ){
				cout << "Separation test failed!" << endl;
				return 0;
			} else {
				cout << "Number of tools validated: [" << tools_Ordered.size() << "]" << endl;

				//For mode 2, publish the pixel differences
				xyzPxMsg.linear.y = tools_Ordered[0].x - ceil(frame.cols/2);
				xyzPxMsg.linear.z = -( tools_Ordered[0].y - ceil(frame.rows/2) );
				cascade_Count++;
				return 1;
			}
		} else {
			cout << "Frame is empty. Returning..." << endl;
			return 0;
		}
	}
	
	
	//NN train functions
	int trainNN(){
		readDataFiles();
		nnetwork = new (nothrow) CvANN_MLP;
		if (nnetwork == nullptr) {
			cout << "Memory could not be allocated for NN!!!" << endl;
			return -1;
		}
		nnetwork -> create(layers , CvANN_MLP::SIGMOID_SYM, 1, 1);
		
		//NN training parameters
		CvANN_MLP_TrainParams paramsNN = CvANN_MLP_TrainParams(
			CvTermCriteria(),//CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,400,0.00001
			CvANN_MLP_TrainParams::BACKPROP,
			0.1,
			0.1);
		
		//Crop the last element of nn_Output_vec
		nnTrain_out = Mat::zeros(nn_Output_vec.size()-1,1,CV_64F);
		for (int i = 0; i < nn_Output_vec.size()-1; i++)
		{
			nnTrain_out.at<double>(i,0) = nn_Output_vec[i];
		}
		//Train
		nnetwork -> train(nnInput_Data_mat, nnTrain_out, Mat(), Mat(), paramsNN);
		cout << "VIZ: NN trained" << endl;
		return 0;
	}

	//Declare
	void detect_circlePub(){
		detect_circle(frame, frame_gray);
	}




private:
	/** @function detectAndDisplay */
	//Detected, validated, overlap checked and sorted tools stored in tools_Ordered public var
	vector<Rect> detectAndDisplay( Mat _frame )
	{
		Mat toolsCroppedTemp = Mat::zeros(min_W,min_L,CV_8U);
        vector<double> toolsColTemp;
        Mat test_input = Mat::zeros(no_of_in_layers,1,CV_64F);
        Mat test_input_real = Mat::zeros(1,no_of_in_layers,CV_64F);

	    //Mat frame_gray(480,640,CV_8U);
	    Mat img_Rect = _frame;
	    Mat nn_Img = _frame;

	    cvtColor( _frame, frame_gray, CV_BGR2GRAY );
	    //equalizeHist( frame_gray, frame_gray );

	    //-- Detect tools
	    casClassifier.detectMultiScale( frame_gray, tools, 1.005, 5, 0|CASCADE_SCALE_IMAGE, Size(0, 0) );

	    for( size_t i = 0; i < tools.size(); i++ ){
	  	    rectangle(_frame, Point(tools[i].x, tools[i].y), Point(tools[i].x + tools[i].width, tools[i].y + tools[i].height), Scalar(255,0,0), 2); 
	  	    Mat toolROI = frame_gray( tools[i] );
	  	    
	  	    printf("i = %d\n", (int)i + 1);

	        //NN transformations
	        toolsCroppedTemp = Mat::zeros(min_W,min_L,CV_8U);
	        resize(frame_gray(tools[i]), toolsCroppedTemp, Size(min_W,min_L));

	        for (int j = 0; j < min_W; j++) {
	        	for (int k = 0; k < min_L; k++) {
	        		toolsColTemp.push_back((double)toolsCroppedTemp.at<uchar>(k,j));
	        	}
	        }

	        data_mean_vec.resize(min_W*min_L);

	        for (int i = 0; i < data_mean_vec.size(); i++) {
	        	toolsColTemp[i] = toolsColTemp[i] - data_mean_vec[i];
	        }

	        test_input = Mat::zeros(no_of_in_layers,1,CV_64F);
	        for (int i = 0; i < no_of_in_layers; i++) {
	        	for (int j = 0; j < toolsColTemp.size(); j++)
	        	{
	        		test_input.at<double>(i,0) = test_input.at<double>(i,0) + pce_mat.at<double>(j,i)*toolsColTemp[j];
	        	}
	        }
	        //cout << "VIZ: NN prediction starting..." << endl;
	        
	        test_input_real = test_input.t();
	        Mat nnResult = Mat(1,1,CV_64F); //NN result is binary (here using int data type).
	        nnetwork -> predict(test_input_real, nnResult);
	        
	        //Show the NN prediction result for the ith tool
	        //cout << nnResult.at<double>(0,0) << endl;
	        if (nnResult.at<double>(0,0) > 0.75) {
	        	rectangle(_frame, Point(tools[i].x, tools[i].y), Point(tools[i].x + tools[i].width, tools[i].y + tools[i].height), Scalar(0,0,255), 2); 
	        	//cout << "VIZ: Tool detected with NN" << endl;
	        }
	        toolsColTemp.clear();
	    }
		
		//Remove overlaps and sort tool sizes
		if ( !calcRectOverlap(tools, _frame) ) {
		 	cout << "Overlap detection failed!!!" << endl;
		}
		
	    //-- Show results
	    imshow( "Ordered tools", _frame );
	    waitKey(2);

	    if (tools.size() > 0) {
	    	//Crop frame_gray for the identification code
		    frame_to_ident = frame_gray(tools[0]);
		    frame_color_ident = _frame(tools[0]);
	    }
	    
	    //Clear tools
	    tools.clear();
	    //Return the sorted tools
	    return tools_Ordered;
	 }



	// this function computes the ratio of two consecutive bounding boxes (NOT CORRECT!)
	float computeRectJoinUnion(const CvRect &rc1, const CvRect &rc2)
	{
	    CvPoint p1, p2;                 
	    p1.x = std::max(rc1.x, rc2.x);
	    p1.y = std::max(rc1.y, rc2.y);

	    p2.x = std::min(rc1.x +rc1.width, rc2.x +rc2.width);
	    p2.y = std::min(rc1.y +rc1.height, rc2.y +rc2.height);

	    float AJoin = abs(p2.x - p1.x)*abs(p2.y - p1.y);
	    float A1 = rc1.width * rc1.height;
	    float A2 = rc2.width * rc2.height;
	    float AUnion = (A1 + A2 - AJoin);                 

	    if( AUnion > 0 ){
	        return (AJoin/AUnion);                  
	    } else{
	        return 0;
	    }
	}
	
	
	//Calculate box overlapping: if two rectangles overlap more than 0.3
	//remove the bigger box
	int calcRectOverlap(vector<Rect> tools, Mat _frame){
		//Calculate overlap and store non-overlapping in tools_Overlap
		vector<Rect> tools_Overlap = tools;
		if (tools.size() >= 2) {
			for (int i = 0; i < tools.size() - 1; i++) {
				for (int j = i+1; j < tools.size(); j++) {
					double Ai = tools[i].width*tools[i].height;
					double Aj = tools[j].width*tools[j].height;
					double Amax = max(Ai,Aj);
					double Amin = max(Ai,Aj);
					if( (tools[i] & tools[j]).area() >= 0.3*Amin){
						if (Amax == Ai) {
							tools_Overlap.erase( remove( tools_Overlap.begin(), tools_Overlap.end(), tools_Overlap[i] ), tools_Overlap.end() );
						}
						else {
							tools_Overlap.erase( remove( tools_Overlap.begin(), tools_Overlap.end(), tools_Overlap[j] ), tools_Overlap.end() );
						}
					}
				}
			}
			//Show non-overlapped detections with green
			for (int i = 0; i < tools_Overlap.size(); i++)
			{
				rectangle(_frame, Point(tools_Overlap[i].x, tools_Overlap[i].y), Point(tools_Overlap[i].x + tools_Overlap[i].width, tools_Overlap[i].y + tools_Overlap[i].height), Scalar(0,255,0), 2);
			}

			//Sort tools based on horizontal pose
			vector<int> horPoseIdx, horPoseVec; //Vector to store the index order and the x-axis order
			vector<Rect> tools_Ovr_Tmp;
			
			//Order tools
			tools_Ordered.clear();
			tools_Ordered.push_back( tools_Overlap[0] );
			for (int i = 1; i < tools_Overlap.size(); i++) {
				vector<Rect>::iterator iter_Order = tools_Ordered.end();
				for (int k = i-1; k > -1; k--)
				{
					if ( (i == 1) && (k == 0) )
					{
						if (tools_Overlap[i].x >= tools_Ordered[k].x){
							tools_Ordered.insert( iter_Order , tools_Overlap[i] );
						} else {
							tools_Ordered.insert( iter_Order-1 , tools_Overlap[i] );
						}
						break;
					}
					if (tools_Overlap[i].x >= tools_Ordered[k].x)
					{
						tools_Ordered.insert( iter_Order , tools_Overlap[i] );
						break;
					} else if (k == 0) {
						tools_Ordered.insert( iter_Order-1 , tools_Overlap[i] );
						break;
					} else {
						iter_Order--;
					}
				}
			}

			//Return the overlap checked and ordered tools in tools_Ordered
			return 1;
		} else {
			cout << "ERR: Number of tools detected not enough for size sorting" << endl;
			return 0;
		}
	}
	
	//Separation test function
	int separationTest(vector<Rect> tools_Ordered){
		for (int i = 0; i < tools_Ordered.size()-1; i++) {
			tools_Sep.push_back( tools_Ordered[i+1].x - tools_Ordered[i].x );
			if ( (tools_Sep.back() < sep_Pixel_min) || (tools_Sep.back() > sep_Pixel_max)) {
				cout << "VIZ: Separation Test: Distance of tool [" << i << "," << i+1 <<"] = " << tools_Sep.back() << endl;
				return 0;
			}
		}
		return 1;
	}
	
	/***************/
	//Hough circle detection
	void detect_circle(Mat img, Mat gray){
		//Function vars
		vector<double> radius_Valve;
		//Check if the frame is empty
		if (img.empty() == 0) {
			//Blur the image to reduce false detections
			GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
			//Vector of 3D vectors to store circles
			vector<Vec3f> circles;
			Vec3f valve_Circle;
			//Hough circles detection
			HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, img.rows/4, 200, 100 );
			//For each circle, draw circle on the image
			for( size_t i = 0; i < circles.size(); i++ )
			{
				Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				radius.push_back( cvRound(circles[i][2]) );
				// draw the circle center
				circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
				// draw the circle outline
				circle( img, center, radiusValve, Scalar(0,0,255), 3, 8, 0 );
				//If more than one circle detected, take the biggest circle
				if ( ( radius.back() >= radius(radius.size()-1) ) && (i > 0) ) {
					valve_Circle = circles[i];
					Point center_Valve(cvRound(valve_Circle[i][0]), cvRound(valve_Circle[i][1]));
					radius_Valve = cvRound(valve_Circle[i][2]);
				}
			}
			//Calculate distance between the first circle center and image center and publish
			if ( valve_Circle.empty() == 0 ) {
				xyzPxMsg.linear.y = center.x - ceil(img.cols/2);
				xyzPxMsg.linear.z = -(center.y - ceil(img.rows/2));
			}
			//Clear the radiusValve vector
			radius.clear();
			
			cout << "Biggest circle radius: " << radius_Valve << endl;
			//Show the image with circles
			imshow( "circles", img );
			waitKey(1);
		}
	}
	
	
	/***************/
	//Read data csv files
	void readDataFiles() {
		std::ifstream ifs1 ("/home/risc/catkin_ws/src/smachine/src/rexmlfiles/data_mean.csv", ifstream::in);
		while (ifs1.good()) {
			getline(ifs1,data_mean);
			std::stringstream sstrm1(data_mean);
			double data_mean_Temp;
			sstrm1 >> data_mean_Temp;
			data_mean_vec.push_back(data_mean_Temp);
			//cout << data_mean_vec[data_mean_vec.size()-1] << endl;
		}
		ifs1.close();
		
		
		/////////////////////////
		std::ifstream ifs11 ("/home/risc/catkin_ws/src/smachine/src/rexmlfiles/PCe.csv", ifstream::in);
		string current_line;
		vector< vector<double> > all_data;
		while (getline(ifs11,current_line)) {
			vector<double> values;
			stringstream temp(current_line);
			string single_value;
			while(getline(temp,single_value,',')){
				// convert the string element to a integer value
				values.push_back(atof(single_value.c_str()));
			}
			// add the row to the complete data vector
			all_data.push_back(values);
		}
		
		// Now add all the data into a Mat element
		pce_mat = Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_64F);
		// Loop over vectors and add the data
		for(int rows = 0; rows < (int)all_data.size(); rows++){
			for(int cols= 0; cols< (int)all_data[0].size(); cols++){
				pce_mat.at<double>(rows,cols) = all_data[rows][cols];
				//cout << pce_mat.at<double>(rows,cols) << endl;
			}
		}
		
		
		std::ifstream ifs20 ("/home/risc/catkin_ws/src/smachine/src/rexmlfiles/w.csv", ifstream::in);
		vector<vector<double> > all_data2;
		while (getline(ifs20,current_line)) {
			vector<double> values;
			stringstream temp(current_line);
			string single_value;
			while(getline(temp,single_value,',')){
				// convert the string element to a integer value
				values.push_back(atof(single_value.c_str()));
			}
			// add the row to the complete data vector
			all_data2.push_back(values);
		}
		
		// Now add all the data into a Mat element
		nnInput_Data_mat = Mat::zeros((int)all_data2.size(), (int)all_data2[0].size(), CV_64F);
		// Loop over vectors and add the data
		for(int rows = 0; rows < (int)all_data2.size(); rows++){
			for(int cols= 0; cols< (int)all_data2[0].size(); cols++){
				nnInput_Data_mat.at<double>(rows,cols) = all_data2[rows][cols];
			}
		}
		//////////////////////////////////////
		
		
		std::ifstream  ifs4 ("/home/risc/catkin_ws/src/smachine/src/rexmlfiles/t.csv", ifstream::in);
		while (ifs4.good()) {
			getline(ifs4,nn_Output);
			std::stringstream sstrm4(nn_Output);
			double nn_Output_Temp;
			sstrm4 >> nn_Output_Temp;
			nn_Output_vec.push_back(nn_Output_Temp);
		}
		ifs4.close();
		
		cout << data_mean_vec.size() << endl;
		cout << pce_mat.size() << endl;
		cout << nnInput_Data_mat.size() << endl;
		cout << nn_Output_vec.size() << endl;
		
	}
};
