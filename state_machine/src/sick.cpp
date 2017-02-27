#include <ros/ros.h>
#include "time.h"
#include "math.h"
#include <iostream>
#include <stdio.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
//Required custom libraries
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace Eigen;

//Math vars
#define PI 3.14159265359


//Public vars
int valve_Detected (0);
int expected_Thres (420);
ros::Time start_Time; //Declare start_Time
int trigger_Sick (1); //Command mode variable
int range_Sick (0); 

int iter_no (0);
int n_Scan (20); //Number of scans by UR
int pt_0 = 4, pt_1 = 7; //First and second points for solving th_1
int n_Dists (20); //Number of distances to be averaged
double dist_Avg (0.0);
int valveSized (0);

//Matrix to store valve sizes
vector<double> M_valve (n_Scan,0.0);
//Vector to store distances
vector<double> v_dist (n_Dists,1000);
//Send the joint positions at the init and final readings
vector<double> Q_joints (6,0); //Q_1 - Q_6 in joint space
vector<double> Q_start_cart (3,0); //Q_1 - Q_6 in cartesian coordinates
vector<double> Q_end_cart (3,0);
double theta_1 (0.0);
double l_side (0.0);

//Matrices to store joint poses for start and end poses
MatrixXd M_start = MatrixXd::Zero(n_Scan,3);
MatrixXd M_end = MatrixXd::Zero(n_Scan,3);



//Trigger callback (from UR)
void cbTrigger(const std_msgs::Int32::ConstPtr& msgTrig)
{
	trigger_Sick = 1;//msgTrig -> data;
}


//Sick range callback
void cbSick(const std_msgs::Int32::ConstPtr& msgSick)
{
	range_Sick = msgSick -> data;
	v_dist[n_Dists-1] = range_Sick;
	rotate(v_dist.begin(),v_dist.begin()+1,v_dist.end());
	//Take the average of the last n_Dists readings
	dist_Avg = accumulate(v_dist.begin(), v_dist.end(), 0)/n_Dists;
	//cout << "Average distance [" << dist_Avg << "]" << endl;
}


//Encoder Callback
void cbJoints(const sensor_msgs::JointState::ConstPtr& msgJoint)
{
	const vector<string> joint_Names = msgJoint -> name;
	if (joint_Names[0] == "ur5_arm_shoulder_pan_joint") {
		Q_joints = msgJoint -> position;
	}
}



//Forward kinematics
vector<double> forwKin(double q1, double q2ur5, double q3ur5){
	// returns x,y,z of wrist (mm)
	double links [4] = {86.900, 111.70, 425.24, 393.94};
	double dz = links[0];
	double l1 = links[1];
	double l2 = links[2];
	double l3 = links[3];
	
	//%%%%%%%%%%%%%%%% From matlab code %%%%%%%%%%%%%%%%
	double q2 = -PI/2.0 - q2ur5;
	double q3 = -(q3ur5 + q2ur5);
	
	double z = dz + l2*cos(q2) + l3*sin(q3);
	double v = l3*cos(q3) - l2*sin(q2);
	
	double y = -l1*cos(-q1) - v*sin(q1);
	double x = -l1*sin(-q1) - v*cos(q1);
	//%%%%%%%%%%%%%%%% From matlab code %%%%%%%%%%%%%%%%
	
	vector<double> cartCoord (3,0);
	cartCoord[0] = x; cartCoord[1] = y; cartCoord[2] = z;
	return cartCoord;
}

	

double getValveSize(){
	//Local vars
	double valve_Ms (0.0);
	//If valve detected start the timer
	if ((dist_Avg < expected_Thres) && (valve_Detected == 0))
	{
		//Start the timer
		start_Time = ros::Time::now();
		//Record the joint positions at reading start point
		Q_start_cart = forwKin(Q_joints[0], Q_joints[1], Q_joints[2]);
		//Record the joint positions at reading start point
		for (size_t i = 0; i < 3; i++) {
			M_start (iter_no,i) = Q_start_cart[i];
		}
		//
		cout << "SICK: Timer started..." << endl;
		valve_Detected = 1; //Do not enter this statement again
		valve_Ms = 0; //Reset
	} else if ((dist_Avg >= expected_Thres) && (valve_Detected == 1)) {
		//End the timer
		ros::Duration dur_onValve = (ros::Time::now() - start_Time);
		valve_Ms = 1000*dur_onValve.toSec(); //Convert to ms and publish
		//Record the joint positions at reading end point
		Q_end_cart = forwKin(Q_joints[0], Q_joints[1], Q_joints[2]);
		//Record the joint positions at reading start point
		for (size_t i = 0; i < 3; i++) {
			M_end (iter_no,i) = Q_end_cart[i];
		}
		//Show the elapsed time in ms
		//cout << "SICK: Time elapsed: " << valve_Ms << endl;
	}
	return valve_Ms;
}









/***********************/
//Gripper interface function. Provides communication between Arduino and SM
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sick_node");
	ros::NodeHandle node_Sick;
	
	
	//Variables published
	std_msgs::Int32 finished_S; //Task finish confirmation published to UR
	finished_S.data = 1; //Start with confirmation
	geometry_msgs::Twist ms_Sick; //Grip commands sent to Arduino

	
	//SUBSCRIBE
	//Sick distance (coming from h/w)
	ros::Subscriber sub_range = node_Sick.subscribe<std_msgs::Int32>("SICK_distance", 100, cbSick);
	//Sick trigger mode (coming from UR)
	ros::Subscriber sub_trig = node_Sick.subscribe<std_msgs::Int32>("cmdtrig_sick", 100, cbTrigger);
	//Encoder Fb (coming from UR)
	ros::Subscriber sub_joints = node_Sick.subscribe<sensor_msgs::JointState>("joint_states", 100, cbJoints);
	
	//PUBLISH
	//Current gripper mode published to SM
	ros::Publisher pub_s_task = node_Sick.advertise<std_msgs::Int32>("sick_mode",100);
	//Pinch number
	ros::Publisher pub_msSick = node_Sick.advertise<geometry_msgs::Twist>("sick_ms",100);
	
	trigger_Sick = 1;
	//Main Loop
	ros::Rate r(100);
	while(ros::ok())
	{
		if (trigger_Sick == 1) {
			double valve_size_ms = getValveSize();
			if (valve_size_ms != 0) {
				finished_S.data = 1;
				valve_Detected = 0;
				if (iter_no < n_Scan) {
					M_valve [iter_no] = valve_size_ms;
					cout << "SICK: Scan [" << iter_no << "] time: " << M_valve [iter_no] << endl;
					iter_no++;
				} else {
					//cout << "SICK: Scan finished!!!!!!!" << endl;
				}
			}
		} else {
			//Sick is supposed to await for command
			finished_S.data = 0;
			//cout << "SICK: Waiting for trigger..." << endl;
		}
		//Find the slope and side
		if ((iter_no == n_Scan) && (valveSized == 0)) {
			//Find the y-z differences between to reading start points
			double y_Diff = M_start (pt_0,1) - M_start (pt_1,1);
			cout << "y_pt0 - y_pt1 = " << y_Diff << endl;
			double z_Diff = M_start (pt_0,2) - M_start (pt_1,2);
			cout << "z_pt0 - z_pt1 = " << z_Diff << endl;
			theta_1 = atan2( z_Diff , y_Diff );
			cout << "theta_1: [" << theta_1 << "]" << endl;
			
			//Find the side
			//Record the y_differences in m_Diff
			vector<double> m_Diff (n_Scan,0);
			for (size_t i = 0; i < n_Scan; i++) {
				m_Diff[i] = M_end (i,1) - M_start (i,1);
			}
			//Find the maximum y_Diff
			double l_max = *max_element(m_Diff.begin(), m_Diff.end()); //Returns the max scanned length
			//Find the side length using geometry
			l_side = l_max * cos(theta_1);
			cout << "SICK: Side length: " << l_side << endl;
			//Do not enter this condition again
			valveSized = 1;
		}
		
		//Publish task finish confirmation
		pub_s_task.publish(finished_S);
		//Publish the current vision mode
		pub_msSick.publish(ms_Sick);
		
		//Spin and sleep
		ros::spinOnce();
		r.sleep();
	}
}
