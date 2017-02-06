#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_msgs/TFMessage.h>
#include <math.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;




class MoveUr {
public:
    //Math vars
    #define PI 3.14159265359
    
    //Joint vars
    vector<double> joint_Pos(6,0);
    vector<double> joint_Vel(6,0);
    vector<double> ur_effort(6,0);
    vector<string> ur_name;
    ros::Duration time_Int(0.0);
    
    //Wrench variables
    double wForce_X(0);
    double wForce_Y(0);
    double wForce_Z(0);
    double wTorque_X(0);
    double wTorque_Y(0);
    double wTorque_Z(0);
    
    double tfMat_TranX(0);
    double tfMat_TranY(0);
    double tfMat_TranZ(0);
    double tfMat_QuatX(0);
    double tfMat_QuatY(0);
    double tfMat_QuatZ(0);
    double tfMat_QuatW(0);
    
    //UR Permission variables
    bool ur_Permission(true);
    
    //Published variables
    control_msgs::FollowJointTrajectoryActionGoal joint_Cmd;
    
    //Function vars
    //main
    int traj_Select = 0;
    
    //moveJoint
    vector<double> Q_1(6,0);
    ros::Duration move_Duration(0.0);
    double motion_Time = 5.0;
    
    //moveCartesian
    double coord_X(0.0);
    double coord_Y(0.0);
    double coord_Z(0.0);
    double coordAng_X(0.0);
    double coordAng_Y(0.0);
    double coordAng_Z(0.0);
    
    //relativeMot
    std_msgs::String strMovePub;





	/*************   FUNCTIONS   **************/
	//Encoder Callback
	void jointCallback(const sensor_msgs::JointState::ConstPtr& msgJoint)
	{
		const vector<string> joint_Names = msgJoint -> name;
		if (joint_Names[0] == "shoulder_pan_joint") {
			joint_Pos = msgJoint -> position;
		    joint_Vel = msgJoint -> velocity;
		    ur_effort = msgJoint -> effort;
		}
	}

	//Wrench Callback
	void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrenchData)
	{
		wForce_X = wrenchData -> wrench.force.x;
		wForce_Y = wrenchData -> wrench.force.y;
		wForce_Z = wrenchData -> wrench.force.z;
	    wTorque_X = wrenchData -> wrench.torque.x;
	    wTorque_Y = wrenchData -> wrench.torque.y;
	    wTorque_Z = wrenchData -> wrench.torque.z;
	}


	//Coordinates Callback
	void coordCallback(const geometry_msgs::Twist::ConstPtr& coordData)
	{
		coord_X = coordData -> linear.x;
		coord_Y = coordData -> linear.y;
		coord_Z = coordData -> linear.z;
	    coordAng_X = coordData -> angular.x;
		coordAng_Y = coordData -> angular.y;
		coordAng_Z = coordData -> angular.z;

		double K_tool = 0.2;
		coord_X = K_tool*coord_X/1000;
		coord_Y = K_tool*coord_Y/1000;
	}

	//Permit Callback
	void permitCallback(const std_msgs::Bool::ConstPtr& permitData)
	{
		ur_Permission = permitData -> data;
	}



private:
	double RtoD( double Radianst )
	{ 
	    return Radianst * (180 / PI); 
	} 

	/*********   DRIVE FUNCTIONS   ***********/

	//MOVE JOINT
	//Moves to input joint position.
	//Input: 6 joint angles. Output: None.
	void moveJoint(double time_Move, double q1, double q2, double q3, double q4, double q5, double q6)
	{
		cout << "ARM: Received joint motion command..." << endl;

	    Q_1[0] = q1;
	    Q_1[1] = q2;
	    Q_1[2] = q3;
	    Q_1[3] = q4;
	    Q_1[4] = q5;
	    Q_1[5] = q6;
	    move_Duration = ros::Duration(time_Move);
	    
	    //Define the waypoints. First is the current position inferred from joint_states topic.
	    //Second is the goal position.
	    int noOfWaypoints = 2;
		joint_Cmd.goal.trajectory.points.resize(noOfWaypoints);

		//Joint names
		joint_Cmd.goal.trajectory.joint_names.resize(6);
		joint_Cmd.goal.trajectory.joint_names[0] = "shoulder_pan_joint";
		joint_Cmd.goal.trajectory.joint_names[1] = "shoulder_lift_joint";
		joint_Cmd.goal.trajectory.joint_names[2] = "elbow_joint";
		joint_Cmd.goal.trajectory.joint_names[3] = "wrist_1_joint";
		joint_Cmd.goal.trajectory.joint_names[4] = "wrist_2_joint";
		joint_Cmd.goal.trajectory.joint_names[5] = "wrist_3_joint";
		
		// First trajectory point
		// Positions
		joint_Cmd.goal.trajectory.points[0].positions.resize(6);
		joint_Cmd.goal.trajectory.points[0].positions[0] = joint_Pos[0];
		joint_Cmd.goal.trajectory.points[0].positions[1] = joint_Pos[1];
		joint_Cmd.goal.trajectory.points[0].positions[2] = joint_Pos[2];
		joint_Cmd.goal.trajectory.points[0].positions[3] = joint_Pos[3];
		joint_Cmd.goal.trajectory.points[0].positions[4] = joint_Pos[4];
		joint_Cmd.goal.trajectory.points[0].positions[5] = joint_Pos[5];
		// Velocities
		joint_Cmd.goal.trajectory.points[0].velocities.resize(6);
		for (size_t j = 0; j < 6; j++) {
			joint_Cmd.goal.trajectory.points[0].velocities[j] = 0.0;
	    }
	    // To be reached 0 second after starting along the trajectory
	    joint_Cmd.goal.trajectory.points[0].time_from_start = ros::Duration(0.0);


	    // Second trajectory point
		// Positions
		joint_Cmd.goal.trajectory.points[1].positions.resize(6);
		joint_Cmd.goal.trajectory.points[1].positions[0] = Q_1[0];
		joint_Cmd.goal.trajectory.points[1].positions[1] = Q_1[1];
		joint_Cmd.goal.trajectory.points[1].positions[2] = Q_1[2];
		joint_Cmd.goal.trajectory.points[1].positions[3] = Q_1[3];
		joint_Cmd.goal.trajectory.points[1].positions[4] = Q_1[4];
		joint_Cmd.goal.trajectory.points[1].positions[5] = Q_1[5];
		// Velocities
		joint_Cmd.goal.trajectory.points[1].velocities.resize(6);
		for (size_t j = 0; j < 6; j++) {
			joint_Cmd.goal.trajectory.points[1].velocities[j] = 0.0;
	    }
	    // To be reached time_Move second after starting along the trajectory
	    joint_Cmd.goal.trajectory.points[1].time_from_start = move_Duration;
	    cout << "Joint angles solved!" << endl;
	    //cout << "ARM: Q_1: [" << Q_1[0] << ", " << Q_1[1] << ", " << Q_1[2] << ", " << Q_1[3] << ", " << Q_1[4] << ", " << Q_1[5] << "]" << endl;
	}






	//MOVE CARTESIAN
	//Moves to input cartesian position.
	//Input: [x,y,z,w,p,r]. Output: None.
	void moveCartesian(double time_Move, double cart_X, double cart_Y, double cart_Z)
	{
	    //Inverse kinematics
		//Declare Variables
		double a[] = {0,-0.425,-0.39225,0,0,0};
		double d[] = {0.089159,0,0,0.10915,0.0956,0.0823};
		double dOff = 0.1117;
		vector<double> q_Inv(6,0);

		double r = sqrt(pow(cart_X,2)+pow(cart_Y,2));
		double s = cart_Z - d[0];
		double D = (pow(s,2)+pow(r,2)-pow(a[1],2)-pow(a[2],2))/(2*a[1]*a[2]);


		double theta_1 = atan2(cart_Y,cart_X) + atan2(dOff , sqrt(pow(cart_X,2)+pow(cart_Y,2)-pow(dOff,2)));
		double theta_3 = atan2(-sqrt(1-pow(D,2)),D);
		double theta_2 = atan2(s,r) - atan2(a[2]*sin(theta_3), a[1]+a[2]*cos(theta_3));

		if (cart_Y < 0)	{
			theta_1 = 2*PI + theta_1;
		}

		double theta_4 = -PI + abs(atan2(sin(theta_2),cos(theta_2))) - theta_3;
		double theta_5 = (-PI/2)-theta_1;
		double theta_6 = 0;

		q_Inv[0] = theta_1;//atan2(sin(theta_1),cos(theta_1));
		q_Inv[1] = atan2(sin(theta_2),cos(theta_2));
		q_Inv[2] = theta_3;//atan2(sin(theta_3),cos(theta_3));
		q_Inv[3] = atan2(sin(theta_4),cos(theta_4));
		q_Inv[4] = atan2(sin(theta_5),cos(theta_5));
		q_Inv[5] = theta_6;

		//Send the goal position in joint angles to moveJoint fcn.
		moveJoint(time_Move, q_Inv[0], q_Inv[1], q_Inv[2], q_Inv[3], q_Inv[4], q_Inv[5]);
	    cout << "ARM: q_Inv: [" << RtoD(q_Inv[0]) << ", " << RtoD(q_Inv[1]) << ", " << RtoD(q_Inv[2]) << ", " << RtoD(q_Inv[3]) << ", " << RtoD(q_Inv[4]) << ", " << RtoD(q_Inv[5]) << "]" << endl;

	}

	//Move based on the camera pizel difference
	void relativeMot(){
		ostringstream coord_X_str, coord_Y_str;
		coord_X_str << coord_X;
		coord_Y_str << coord_Y;

		string strMove = "movel(pose_trans(get_forward_kin(),p[";
		strMove = strMove.append(coord_X_str.str());
		strMove = strMove.append(",");
		strMove = strMove.append(coord_Y_str.str());
		strMove = strMove.append(",0,0,0,0]),a=0.02,v=0.02,t=0,r=0)");
		//cout << strMove << endl;
		strMovePub.data = strMove;
	}


	//Move to specified relative XYZ
	//XYZ is based on the tool frame!
	void moveTool(double distX, double distY, double distZ){
		ostringstream dist_X_str, dist_Y_str, dist_Z_str;
		dist_X_str << distX; dist_Y_str << distY; dist_Z_str << distZ;

		string strMove = "movel(pose_trans(get_forward_kin(),p[";
		strMove = strMove.append(dist_X_str.str());
		strMove = strMove.append(",");
		strMove = strMove.append(dist_Y_str.str());
		strMove = strMove.append(",");
		strMove = strMove.append(dist_Z_str.str());
		strMove = strMove.append(",0,0,0]),a=0.02,v=0.02,t=0,r=0)");
		//cout << strMove << endl;
		strMovePub.data = strMove;
	}





/*****************************************
/*****************************************
/***********   MAIN FUNCTION   ***********
/*****************************************
*****************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "moveUr");
	ros::NodeHandle node_MoveUr;
	ros::Rate loop_rate(20);
	
	

	//Advertise velocity command publisher
    
	
	ros::Time time_MotInit = ros::Time::now();
	motion_Time = 10;
	int modeUR = 0;
	int initialMove = 0;
	
	//Main loop
	while (ros::ok())
	{
		switch(modeUR){
			case(0):
	    		if (ros::Time::now() - time_MotInit <= ros::Duration(ceil(motion_Time) + 0.5)){
    				if (initialMove < 2){
						cout << "ARM: Motion finished..." << endl;
			    		time_MotInit = ros::Time::now();
			    		moveCartesian(motion_Time, -0.4, -0.08, 0.35);
			    		commandPub.publish(joint_Cmd);
			    		initialMove++;
					} else {
						modeUR = 1;
					}
		    	}
		    	break;
			case(1):
				if (ros::Time::now() - time_MotInit <= ros::Duration(ceil(motion_Time) + 0.5)){
					cout << "ARM: Moving to coord= " << coord_X << ", " << coord_Y << endl;
		    		time_MotInit = ros::Time::now();
		    		relativeMot();
		    		pubUrscript.publish(strMovePub);
				} else {
					cout << "ARM: is moving..." << endl;
			    }
			    break;
			case(2):
				if (ros::Time::now() - time_MotInit <= ros::Duration(ceil(motion_Time) + 0.5)){
					cout << "ARM: Moving to coord= " << coord_X << ", " << coord_Y << ", " << coord_Z << endl;
		    		time_MotInit = ros::Time::now();
		    		moveTool(0.5,0,0);
		    		pubUrscript.publish(strMovePub);
				} else {
					cout << "ARM: is moving..." << endl;
			    }
			    break;
		}

		cout << "mode:" << modeUR <<endl;

/*
		if (initialMove < 2){
			if (ros::Time::now() - time_MotInit <= ros::Duration(ceil(motion_Time) + 0.5)){
				cout << "ARM: Motion finished..." << endl;
	    		time_MotInit = ros::Time::now();
	    		moveCartesian(motion_Time, -0.4, -0.08, 0.35);
	    		commandPub.publish(joint_Cmd);
	    		initialMove++;
			} else {
				cout << "ARM: is moving..." << endl;
		    }
		} else {
			if (ros::Time::now() - time_MotInit <= ros::Duration(ceil(motion_Time) + 0.5)){
				cout << "ARM: Moving to coord_X= " << coord_X << ", " << coord_Y << endl;
	    		time_MotInit = ros::Time::now();
	    		relativeMot();
	    		pubUrscript.publish(strMovePub);
			} else {
				cout << "ARM: is moving..." << endl;
		    }
		}
*/
		sleep(ceil(motion_Time) + 0.2);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
