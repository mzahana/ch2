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

class CascadeDet
{
public:
	//Global vars
	String cascade_name, video_name, window_name, data_mean, pceNN, nnInput_Data, nn_Output;
	CascadeClassifier casClassifier;
	vector<Rect> tools, _lasttools;
	
	//Neural net vars
	CvANN_MLP* nnetwork;
	Mat layers;
	int min_L, min_W, no_of_in_layers, no_of_hid_layers, no_of_out_layers;
	vector<double> data_mean_vec, nn_Output_vec;
	
	//Frame matrices
	Mat frame, frame_gray, frame_to_ident, frame_color_ident;
	Mat pce_mat, nnInput_Data_mat, nnInput_Data_matTr, nnTrain_out;
	
	
	//Publish vars (automatically initializes with zero vals)
	geometry_msgs::Twist xyzPxMsg;

	

	//Constructor
	CascadeDet(){
		cascade_name = "/home/risc/catkin_ws/src/husky_vision/src/rexmlfiles/toolDetector5_1(3).xml";
		min_L = 30, min_W = 30;
		no_of_in_layers = 18; no_of_hid_layers = 36; no_of_out_layers = 1;
		layers = (Mat_<int>(1,3) << no_of_in_layers,no_of_hid_layers,no_of_out_layers);
	}


	//Read data csv files
	void readDataFiles() {
		std::ifstream ifs1 ("/home/risc/catkin_ws/src/husky_vision/src/rexmlfiles/data_mean.csv", ifstream::in);
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
		std::ifstream ifs11 ("/home/risc/catkin_ws/src/husky_vision/src/rexmlfiles/PCe.csv", ifstream::in);
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


		std::ifstream ifs20 ("/home/risc/catkin_ws/src/husky_vision/src/rexmlfiles/w.csv", ifstream::in);
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


		std::ifstream  ifs4 ("/home/risc/catkin_ws/src/husky_vision/src/rexmlfiles/t.csv", ifstream::in);
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
		//cvtColor( cv_ptr -> image, frame_gray, COLOR_BGR2GRAY );
	}


	void detectProcess(){
		if( !casClassifier.load( cascade_name ) ){ printf("--(!)Error loading xml file\n"); };
		cout << "VIZ: Frame reading started..." << endl;
		if ( frame.empty() == 0 ) {
			detectAndDisplay( frame );
		} else {cout << "Frame is empty. Returning..." << endl;}
	}
	
	
	//NN train functions
	void trainNN(){
		readDataFiles();
		nnetwork = new CvANN_MLP;
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

		nnetwork -> train(nnInput_Data_mat, nnTrain_out, Mat(), Mat(), paramsNN);
		cout << "VIZ: NN trained" << endl;
	}




private:
	/** @function detectAndDisplay */
	void detectAndDisplay( Mat _frame )
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
	    casClassifier.detectMultiScale( frame_gray, tools, 1.005, 1, 0|CASCADE_SCALE_IMAGE, Size(0, 0) );

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
	        	cout << "VIZ: [" << i << "] " << test_input.at<double>(i,0) << endl;
	        }
	        cout << "VIZ: NN prediction starting..." << endl;
	        
	        test_input_real = test_input.t();
	        Mat nnResult = Mat(1,1,CV_64F); //NN result is binary (here using int data type).
	        nnetwork -> predict(test_input_real, nnResult);
	        

	        cout << nnResult.at<double>(0,0) << endl;
	        if (nnResult.at<double>(0,0) > 0.75) {
	        	rectangle(_frame, Point(tools[i].x, tools[i].y), Point(tools[i].x + tools[i].width, tools[i].y + tools[i].height), Scalar(0,0,255), 2); 
	        	cout << "VIZ: Tool detected with NN" << endl;
	        }
	        toolsColTemp.clear();
	    }
	    
	    //-- Show results
	    imshow( "Image rectangle", _frame );
	    waitKey(2);

	    if (tools.size() > 0) {
	    	//Crop frame_gray for the identification code
		    frame_to_ident = frame_gray(tools[0]);
		    frame_color_ident = _frame(tools[0]);
	    }
	    
	    //Clear tools
	    tools.clear();
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
};
