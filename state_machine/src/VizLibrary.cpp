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
#include "std_msgs/String.h"
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;




class VizLibrary
{
public:
	//Global vars
	String cascade_name, video_name, window_name, data_mean, pceNN, nnInput_Data, nn_Output;
	CascadeClassifier casClassifier;
	vector<Rect> tools, tools_Overlap, tools_Phase1, tool_Phase2, tools_Neural, tools_Ordered; //Vectors to store the processed tools
	double rect_Overlap_rat;
	int cascade_Count;
	string mode_V_cmd;


	//IdentSize vars
	int level_Count;
	int n_Level; //total desired # of detections to store tool tip centers and joint poses
	vector<double> M_tools_avgY; //Vector to store average lengths of tools (needed outside of this class)
	int idx_SmallestTool; //Index of the smallest tool
	double smallestToolSize; //Length of the smallest detected tool
	int i_Level, noOfTools;
	MatrixXd M_tools_X; //Matrix to store tools centers X (assuming six tools detected)
	MatrixXd M_tools_Y; //Matrix to store tools centers Y
	MatrixXd M_joints; //Matrix to store the joint poses
	//Joint pose
	vector<double> joint_Pose;
	
	
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
	int pixelThres = 2;
	//Vector of 3D vectors to store circles
	vector<Vec3f> circles;
	
	//Publish vars (automatically initializes with zero vals)
	geometry_msgs::Twist xyzPxMsg;

	

	//Constructor
	VizLibrary(){
		//Load the cascade classifier
		cascade_name = "/home/risc/catkin_ws/src/smach_trial/src/rexmlfiles/toolDetector6_1(1).xml";
		if( !casClassifier.load( cascade_name ) ){ printf("--(!)Error loading xml file\n"); };
		//Parameter initializations
		min_L = 36, min_W = 36;
		no_of_in_layers = 27; no_of_hid_layers = 27; no_of_out_layers = 1;
		layers = (Mat_<int>(1,3) << no_of_in_layers,no_of_hid_layers,no_of_out_layers);
		rect_Overlap_rat = 0.3;
		
		//Separation check vars: Set based on distance from panel
		sep_Pixel_min = 40;
		sep_Pixel_max = 150;
		//Cascade detection number of successful 6-tool-validation
		cascade_Count = 0;
		
		//Ident variables
		level_Count = 0;
		n_Level = 10; i_Level = 1;
		noOfTools = 6;
		M_tools_X = MatrixXd::Zero(n_Level,noOfTools);
		M_tools_Y = MatrixXd::Zero(n_Level,noOfTools);
		M_joints = MatrixXd::Zero(n_Level,6); //6 is the number of joints
		//Initialize the joint poses
		for (int i = 0; i < joint_Pose.size(); i++){ joint_Pose[i] = 0; }
	}
	
	
	//Vision mode callback
	void modeCb(const std_msgs::String::ConstPtr& msgMode)
	{
		mode_V_cmd = msgMode -> data;
	}
	
	
	
	//Image callback
	void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("VIZ: detectionOpenCV::main.cpp::cv_bridge exception: %s", e.what());
			return;
		}
		frame = cv_ptr -> image;
		cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
	}

	
	//Cascade detection function
	int detectProcess(){
		cout << "VIZ: Frame reading started..." << endl;
		if ( frame.empty() == 0 ) {
			tools_Ordered = detectAndDisplay( frame );
			//Apply separation test
			if( !separationTest(tools_Ordered) ){
				cout << "VIZ: Separation test failed!" << endl;
				return 0;
			} else {
				cout << "VIZ: Number of tools validated: [" << tools_Ordered.size() << "]" << endl;
				//Publish the pixel differences
				xyzPxMsg.linear.y = tools_Ordered[0].x - ceil(frame.cols/2);
				xyzPxMsg.linear.z = -( tools_Ordered[0].y - ceil(frame.rows/2) );
				cascade_Count++;
				return 1;
			}
		} else {
			cout << "VIZ: Frame is empty. Returning..." << endl;
			return 0;
		}
	}



	//Cascade detection and size identification function
	int detectAndIdent(){
		cout << "VIZ: Frame reading started..." << endl;
		if ( frame.empty() == 0 ) {
			//Run Cascade 1
			tools_Phase1 = detectCascade( frame );
			
			//Overlap detection (output: tools_overlap)
			//If number of tools > 1, run overlap detection (keep the largest)
			if (tools_Phase1.size() >= 2) {
				//Remove overlaps
				if ( !calcRectOverlap(tools, frame) ) {
					cout << "VIZ: Overlap detection failed!!!" << endl;
				}
			}
			
			//Run Cascade2
			tools_Phase2 = detectCascade( frame , tools_Overlap );
			
			//Run NN on tools_Phase2
			tools_Neural = nnTools( frame , tools_Phase2 );
			
			//Sort the NN-identified tools
			tools_Ordered = sortTools( tools_Neural );
			
			//Apply separation test
			if( !separationTest(tools_Ordered) ){
				cout << "VIZ: Separation test failed!" << endl;
				return 0;
			} else {
				cout << "VIZ: Number of tools validated: [" << tools_Ordered.size() << "]" << endl;
				//If 6 tools are detected and passed separation test, Identify sizes
				if ( tools_Ordered.size() == noOfTools ){
					level_Count = findLevels(tools_Ordered);
					return 1;
				} else {
					return 0;
				}		
			}
		} else {
			cout << "VIZ: Frame is empty. Returning..." << endl;
			return 0;
		}
		
		
		//Show non-overlapping tools
		imshow( "VIZ: Ordered tools", frame );
		waitKey(2);
	}
	
	
	//NN train functions
	int trainNN(){
		readDataFiles();
		nnetwork = new (nothrow) CvANN_MLP;
		if (nnetwork == nullptr) {
			cout << "VIZ: Memory could not be allocated for NN!!!" << endl;
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



	//Encoder Callback
	void jointCallback(const sensor_msgs::JointState::ConstPtr& msgJoint)
	{
		const vector<string> joint_Names = msgJoint -> name;
		if (joint_Names[0] == "ur5_arm_shoulder_pan_joint") {
			joint_Pose = msgJoint -> position;
		}
	}




private:
	/** @function detectCascade */
	//Detected, validated, overlap checked and sorted tools stored in tools_Ordered public var
	//Cascade detection function
	vector<Rect> detectCascade( Mat frame_gray )
	{
	    //Run Cascade to detect tooltips
		casClassifier.detectMultiScale( frame_gray, tools, 1.005, 5, 0|CASCADE_SCALE_IMAGE, Size(0, 0) );
		return tools;
	}
	
	
	
	/** @function detectCascade_second */
	//Second Cascade function
	vector<Rect> detectCascade_second( Mat frame_gray , vector<Rect> tools_Phase1 ) {
		//Clear tools vector (output) before filling it
		tools.clear();
		//Local vars
		vector<Rect> tools_Phase2_tmp;
		//Loop among the Phase_1 tools
		for( size_t i = 0; i < tools_Phase1.size(); i++ ){
			//Run Cascade inside the detected tooltips
			Mat toolROI = frame_gray( tools_Phase1[i] );
			casClassifier.detectMultiScale( toolROI, tools_Phase2_tmp, 1.005, 1, 0|CASCADE_SCALE_IMAGE, Size(0, 0) );
			//If tool(s) detected, append all at the end of the "tools"
			if (tools_Phase2_tmp.empty() == 0) {
				//Add the original (Phase1 tool) to tools
				tools.push_back( tools_Phase1[i] );
				//Add all detected tools to tools vector
				for (int j = 0; j < tools_Phase2_tmp.size(); j++) {
					//Calculate the actual tool_Phase2 coordinate
					tools_Phase2_tmp[j].x += tools_Phase1[i].x;
					tools_Phase2_tmp[j].y += tools_Phase1[i].y;
					tools.push_back( tools_Phase2_tmp[j] );
				}
			} else {
				//Add "only" the original (Phase1 tool) to tools if nothing detected inside it
				tools.push_back( tools_Phase1[i] );
			}
		}
		return tools;
	}
	
	
	
	/** @function nnTools */
	//Neural function
	vector<Rect> nnTools( Mat _frame , vector<Rect> tools )
	{
		//NN private parameters
		Mat toolsCroppedTemp = Mat::zeros(min_W,min_L,CV_8U);
		vector<double> toolsColTemp;
		Mat test_input = Mat::zeros(no_of_in_layers,1,CV_64F);
		Mat test_input_real = Mat::zeros(1,no_of_in_layers,CV_64F);
		Mat img_Rect = _frame;
		Mat nn_Img = _frame;
		vector<Rect> tools_Neural_local;
		
		//Run NN for each rectangle
	    for( size_t i = 0; i < tools.size(); i++ ){
			//Draw rectangle around tooltips
	  	    rectangle(_frame, Point(tools[i].x, tools[i].y), Point(tools[i].x + tools[i].width, tools[i].y + tools[i].height), Scalar(255,0,0), 2);
	  	    
	  	    printf("i = %d\n", (int)i + 1);

	        //NN transformations
	        toolsCroppedTemp = Mat::zeros(min_W,min_L,CV_8U);
	        resize(frame_gray(tools[i]), toolsCroppedTemp, Size(min_W,min_L));

	        for (int j = 0; j < min_W; j++) {
	        	for (int k = 0; k < min_L; k++) {
	        		toolsColTemp.push_back((double)toolsCroppedTemp.at<uchar>(k,j));
	        	}
	        }
			//Crop the last extra entries of data_mean_vec
	        data_mean_vec.resize(min_W*min_L);
			//Normalization
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
		
		//Store the NN identified tools in tools_Neural
		tools_Neural_local = tools;
	    //Clear tools
	    tools.clear();
	    //Return the sorted tools
	    return tools_Neural_local;
	}

	
	/** @function calcRectOverlap */
	//Calculate box overlapping: if two rectangles overlap more than rect_Overlap_rat
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
					double Amin = min(Ai,Aj);
					//Erase the smaller of two overlapping tools
					if( (tools[i] & tools[j]).area() >= rect_Overlap_rat*Amin){
						if (Amax == Aj) {
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
				int colorRect [3]; colorRect[0] = 0; colorRect[1] = 255; colorRect[2] = 0;
				drawRect( frame, tools_Overlap, colorRect );
			}
			return 1;
		} else {
			cout << "ERR: Number of tools detected not enough for overlap test!!!" << endl;
			return 0;
		}
	}
	
	
	/** @function sortTools */
	//Sort tools based on horizontal pose
	vector<Rect> sortTools ( Mat tools_for_sort ) {
		//Local vars
		vector<Rect> tools_Ordered_local;
		tools_Ordered_local.push_back( tools_for_sort[0] );
		//Loop among the input tools
		for (int i = 1; i < tools_for_sort.size(); i++) {
			vector<Rect>::iterator iter_Order = tools_Ordered_local.end();
			for (int k = i-1; k > -1; k--)
			{
				if ( (i == 1) && (k == 0) )
				{
					if (tools_for_sort[i].x >= tools_Ordered[k].x){
						tools_Ordered_local.insert( iter_Order , tools_for_sort[i] );
					} else {
						tools_Ordered_local.insert( iter_Order-1 , tools_for_sort[i] );
					}
					break;
				}
				if (tools_for_sort[i].x >= tools_Ordered_local[k].x)
				{
					tools_Ordered_local.insert( iter_Order , tools_for_sort[i] );
					break;
				} else if (k == 0) {
					tools_Ordered_local.insert( iter_Order-1 , tools_for_sort[i] );
					break;
				} else {
					iter_Order--;
				}
			}
		}
		
		//Return the overlap checked and ordered tools in tools_Ordered
		return tools_Ordered_local;
	}
	
	

	/** @function separationTest */
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



	/** @function findLevels */
	//Finds the vertical levels of tooltip centers
	//Store the detected centers and joint positions for n_Level # of frames
	int findLevels(vector<Rect> tools_Level){
		//Store the values
		for (int j = 0; j < noOfTools; j++) {
			M_tools_X (i_Level-1,j) = tools_Level[j].x + 0.5*tools_Level[j].width;
			M_tools_Y (i_Level-1,j) = tools_Level[j].y + 0.5*tools_Level[j].height;
		}
		cout << "VIZ: Tooltip center heights: " << M_tools_Y << endl;
		cout << "VIZ: joint_Pose.size()s: " << joint_Pose.size() << endl;
		//Store the joint configurations where 6 tools detected
		for (int j = 0; j < joint_Pose.size(); j++) {
			M_joints (i_Level-1,j) = joint_Pose[j];
		}
		
		//If i_Level = n_Level, calculate y-axis lengths and order
		if (i_Level == n_Level) {
			for (int j = 0; j < noOfTools; j++) {
				double M_Ysum = 0;
				for (int i = 0; i < n_Level; i++){ M_Ysum = M_Ysum + M_tools_Y (i,j); }
				M_tools_avgY.push_back( M_Ysum/((double)n_Level) ); //Length of the jth tool's center (robot z-axis)
			}
			//Show the tools lengths (the order is the same as of tools_Ordered)
			for (int i = 0; i < noOfTools; i++)	{
				cout << "VIZ: Tooltip center heights: " << endl;
				cout << "[" << i << "] : " << M_tools_avgY[i] << endl;
			}
			//Find and show the smallest tool
			smallestToolSize = M_tools_avgY[0];
			idx_SmallestTool = 0;
			for (int i = 0; i < noOfTools; i++)	{
				if (smallestToolSize >= M_tools_avgY[i]) {
					smallestToolSize = M_tools_avgY[i];
					idx_SmallestTool = i;
				}
			}
			cout << "Smallest tool: " << idx_SmallestTool << ", Size: " << smallestToolSize << endl;
		}
		
		//Iterate
		i_Level++;
		//Return the current i_Level for checking stopping time of this function
		return i_Level-1;
	}


	/** @function detect_circle */
	//Hough circle detection
	void detect_circle(Mat img, Mat gray){
		//Function vars
		vector<double> radius;
		Point center_Valve;
		//Check if the frame is empty
		if (img.empty() == 0) {
			//Blur the image to reduce false detections
			GaussianBlur( gray, gray, Size(9, 9), 2, 2 );
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
				circle( img, center, radius.back(), Scalar(0,0,255), 3, 8, 0 );
				if ( i == 0 ){ center_Valve = center; }
				//If more than one circle detected, take the biggest circle
				if ( ( radius.back() >= radius[radius.size()-1] ) && (i > 0) ) {
					center_Valve = center;
					//radius_Valve.push_back( cvRound(valve_Circle[i][2]) );
				}
			}
			//Calculate distance between the first circle center and image center and publish
			if ( circles.empty() == 0 ) {
				xyzPxMsg.linear.y = center_Valve.x - ceil(img.cols/2);
				xyzPxMsg.linear.z = -(center_Valve.y - ceil(img.rows/2));
			}
			//Clear the radiusValve vector
			radius.clear();
			
			//cout << "Biggest circle radius: " << radius_Valve << endl;
			//Show the image with circles
			imshow( "circles", img );
			waitKey(1);
		}
	}
	
	
	/** @function drawRect */
	//Rectangle drawing function
	void drawRect( Mat frame , vector<Rect> tools , int colorOrder ) {
		rectangle(frame, Point(tools[i].x, tools[i].y), Point(tools[i].x + tools[i].width, tools[i].y + tools[i].height), Scalar(colorOrder[0],colorOrder[1],colorOrder[2]), 2);
	}



	/** @function readDataFiles */
	//Read data csv files
	void readDataFiles() {
		std::ifstream ifs1 ("/home/risc/catkin_ws/src/smach_trial/src/rexmlfiles/data_mean.csv", ifstream::in);
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
		std::ifstream ifs11 ("/home/risc/catkin_ws/src/smach_trial/src/rexmlfiles/PCe.csv", ifstream::in);
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
		
		
		std::ifstream ifs20 ("/home/risc/catkin_ws/src/smach_trial/src/rexmlfiles/w.csv", ifstream::in);
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
		
		
		std::ifstream  ifs4 ("/home/risc/catkin_ws/src/smach_trial/src/rexmlfiles/t.csv", ifstream::in);
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
