#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
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



using namespace std;
using namespace cv;


class IdentifySize
{
public:
	vector<int> idx_BW, idx_W, idx_B;


	IdentifySize(){
		
	}
	
	int filterImg(Mat img_gray, Mat img_color){
		if (img_gray.empty() == 0) {
			findLength(img_gray, img_color);
			cout << "Running Identification..." << endl;
			return 0;
		} else {cout << "No image for Identification" << endl;return -1;}
	}

private:
	void findLength(Mat img_gray, Mat img_color){
		//Convert to black-white
		Mat img_BW = img_gray > 190;
		cout << "flanksdn" << (int)img_BW.at<uchar>(100,100) << endl;
		Mat img_BW_show = img_BW;
		for (int i = 0; i < img_gray.rows; i++) {
			for (int j = 0; j < img_gray.cols; j++) {
				if((double)img_BW.at<uchar>(i,j) > 0){img_BW.at<uchar>(i,j) = 1;}
			}
		}

		int shift_length = 6;
		vector<double> coef = {0.1,0.1585,0.2512,0.3981,0.6310,1};
		Mat B = Mat::zeros(img_BW.rows,img_BW.cols,CV_64F);
		Mat W = Mat::zeros(img_BW.rows,img_BW.cols,CV_64F);
		int row_in = 10;
		int col_in = 10;



		for (int i = row_in; i < img_BW.rows-row_in-1; i++)
		{
			for (int j = col_in; j < img_BW.cols-col_in; j++)
			{
				int count_1 = 0;
				for (int k = 0; k < 4; k++) {
					if((int)img_BW.at<uchar>(i,j-k) == 1){count_1++;}
				}
				if((int)img_BW.at<uchar>(i+1,j) == 1){count_1++;}
				if (count_1 == 5){
					for (int k = 0; k < shift_length; k++) {
						B.at<double>(i,j) = B.at<double>(i,j) + coef[k]*(1 - (double)img_BW.at<uchar>(i,j+k+1));
					}
				}

				
				//If the black condition is satisfied, check white condn
				if (B.at<double>(i,j) > 0)
				{
					for (int s = j+shift_length; s < img_BW.cols-col_in-1; s++)	{
						int count_2 = 0;
						for (int k = 0; k < 4; k++) {
							if((int)img_BW.at<uchar>(i,j+k) == 1){count_2++;}
						}
						if((int)img_BW.at<uchar>(i-1,j) == 1){count_2++;}
						//Check if on that row there is another W>0 before the sth column.
						double W_check = 0;
						for (int y = 0; y < s; y++)	{W_check = W_check + W.at<double>(i,y);}
						if ( (W_check == 0) && (count_2 == 5) ) {
							for (int h = 0; h < shift_length; h++){
								W.at<double>(i,s) = W.at<double>(i,s) + coef[h]*(1 - (double)img_BW.at<uchar>(i,s-h-1));
							}
						}
					}
				}
				
			}
		}


		//Find the max idx and value of W
		vector<double> max_B, max_W;
		vector<int> idx_B, idx_W;
		for (int i = 0; i < W.rows; i++) {
			max_W.push_back(0); idx_W.push_back(0);
			for (int j = 0; j < W.cols; j++) {
				if (W.at<double>(i,j) > max_W.back()) {
					max_W.back() = W.at<double>(i,j);
					idx_W.back() = j;
				}
			}
			//Zero all entries of B(i,:) after W>0 detected
			if (max_W[i] > 0) {
				for (int r = idx_W[i]; r < W.cols; r++) {
					B.at<double>(i,r) = 0;
				}
			}
		}

		
		//Find the max idx and value of B
		for (int i = 0; i < B.rows; i++) {
			max_B.push_back(0); idx_B.push_back(0);
			for (int j = 0; j < B.cols; j++) {
				if (B.at<double>(i,j) > max_B[i]) {
					max_B.back() = B.at<double>(i,j);
					idx_B.back() = j;
				}
			}
		}


		//Eliminate the rows B>W
		for (int i = 0; i < max_B.size(); i++) {
			if (max_B[i] >= max_W[i]){max_B[i] = 0; max_W[i] = 0;}			
		}

		//Show calculated lines
		for (int i = 0; i < img_color.rows; i++)
		{
			Point pt_1( idx_B[i] , i );
			Point pt_2( idx_W[i] , i );
			//cout << "i: [" << i << "]  idx_B[" << idx_B[i] << "];  idx_W[" << idx_W[i] << endl;
			line(img_color, pt_1, pt_2, (255,0,0), 1);
		}
		imshow("Filtered",img_color);
		waitKey(3);

		for (int i = 0; i < img_BW_show.rows; i++) {
			for (int j = 0; j < img_BW_show.cols; j++) {
				if ((int)img_BW_show.at<uchar>(i,j) == 1) {
					img_BW_show.at<uchar>(i,j) = 255;
				}
			}
		}

		imshow("BW image",img_BW_show);
		waitKey(3);
	}
};