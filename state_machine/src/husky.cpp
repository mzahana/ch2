#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <unistd.h>
#include <vector>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/Range.h"


using namespace std;

/*****************************************
/*****************************************
/************   GLOBAL VARS   ************
/*****************************************
*****************************************/

//Math varsf
#define PI 3.14159265359
double deg2rad = 0.017453292519943295;

//State Machine vars
std_msgs::Int32 mode_SM;

//Scan vars
float range_Middle(0);
float omega_Des(0);
float first_LaserNonZero(0);
std::vector<float> laser_Read_real(540,0);
std::vector<float> laser_Read(540,0);
std::vector<float> laser_Read_avg(540,0);
vector< vector<float> > laser_vector(540, vector<float>(10)); 
int num_laser(0);
geometry_msgs::Twist vel_Cmd;


//Mode variables
double err_w_mode2(0);
int count_mode2(0);

//Lidar vars
float saturate_Lidar(4);
float max_range_prox(3);
//int conesize(10);
int length_laserScn(0); // Number of valid values in the
//Mode 9 count variable
int COUNT_Delay = 0;
 
//Function vars
float K_p(0.1); //Proportional gain for angular ctr
float first_Deg(0);
float cstSpd_X(0.05);
float minDist_Find(1);
float maxDist_Find(5);
float minDist_Rotate(1);
float maxDist_Rotate(4);
float dist_FindToRotate(2);
float dist_GpsToFind(10);
double count_mode41(0);

//Gps vars

//double ptTarget_Long [4]={39.1226263333,39.122519,39.1224123333,39.1225241667};
//double ptTarget_Latt [4]={22.322875, 22.3230868333,22.3230405, 22.3228946667};

double ptTarget_Long [3]={39.1225171667,39.1224298333,39.1225756667};
double ptTarget_Latt [3]={22.3222653333,22.3223658333, 22.3224771667};

double gps_Dist(0);
double gps_Bear(0);
double ptCur_Longt(0);
double ptCur_Latt(0);
double ptCur_Altt(0);
int count_gpspt = 0;

//Imu vars
double magnet_X(0);
double magnet_Y(0);
double magnet_Z(0);
double magnet_W(0);
double gyro_X(0);
double gyro_Y(0);
double gyro_Z(0);
double accel_X(0);
double accel_Y(0);
double accel_Z(0);
double roll_Imu(0);
double pitch_Imu(0);
double yaw_Imu(0);


//Sonar vars
double snr_Range_1(0);
double snr_Range_2(0);
double snr_Range_3(0);
double snr_Range_2_high(0);
double snr_Range_front_Left(0);
double snr_Range_front_Right(0);
double snr1_Read(0), snr2_Read(0), snr3_Read(0), snr4_Read(0), snr5_Read(0), snr6_Read(0);
int num_snr1(0), num_snr2(0), num_snr3(0), num_snr4(0), num_snr5(0), num_snr6(0);
double max_snr_rng(4);

//Tera vars
double tera_1(0);
double tera_2(0);
double tera_3(0);
double tera_4(0);
const double maxTera_Read(2000);
std::vector<float> tera_Read(8,0);


//Turn vars
double yaw_Start(0);
double error_Yaw(0);


//Approach vars
double minDistTo_Box(0);
int flag_init_vision(0);


//Mode switch
int mode = 0;
int cnt_FinalAlign = 0;
ros::Time beginTime;
ros::Duration durationTime;
int ctr_Time(0);
int ctr_Yaw(0);


//Husky vars
double huskyLength_Dif = 500;
float cmdLin_X(0.0); //Husky Speed LinearX
float cmdLin_Y(0.0); //Husky Speed LinearY
float cmdAng_Z(0.0); //Husky Speed AngularZ
double satLin_X(1); //Maximum Speed LinearX
double satAng_Z(0.25); //Maximum Speed AngularZ




/*****************************************
/*****************************************
/*************   FUNCTIONS   *************
/*****************************************
*****************************************/
int count_laser(0);
//Laser Scan Callback
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msgScan)
{
	float laserAngleMin = msgScan -> angle_min;
	float laserAngleMax = msgScan -> angle_max;
	float laserAngleIncr = msgScan -> angle_increment;
	int middleIdx = ceil((laserAngleMax-laserAngleMin)/(2*laserAngleIncr));
	range_Middle = msgScan -> ranges[middleIdx];
	laser_Read_real= msgScan -> ranges;
	//laser_vector[num_laser] = laser_Read_real;        
}




//Gps Callback
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsData)
{
	ptCur_Longt = gpsData -> longitude;
	ptCur_Latt = gpsData -> latitude;
	//std::cout << "Long: " << ptCur_Longt << ", Latt: " << ptCur_Longt << std::endl;
}



//Gps Process
void gpsProcess(double ptTar_Longt,double ptTar_Latt)
{
	
	
	double latDiff = (ptTar_Latt - ptCur_Latt);
	double longDiff = (ptTar_Longt - ptCur_Longt);
	double R = 6371;
	double a = pow(sin(0.5*latDiff* deg2rad),2)+(cos(ptCur_Latt* deg2rad)*cos(ptTar_Latt* deg2rad)*pow(sin(0.5*longDiff* deg2rad),2));
	double c = 2*atan2(sqrt(a),sqrt(1-a));
	gps_Dist = R*c*1000;
	double yBear = sin(longDiff*deg2rad)*cos(ptTar_Latt*deg2rad);
	double xBear = cos(ptCur_Latt*deg2rad)*sin(ptTar_Latt*deg2rad)-sin(ptCur_Latt*deg2rad)*cos(ptTar_Latt*deg2rad)*cos(longDiff*deg2rad);
        gps_Bear = -atan2(yBear,xBear);
	
	// Convert gps_Bear from [0,2pi) to (-pi,pi]
	gps_Bear = atan2(sin(gps_Bear),cos(gps_Bear));
	std::cout << "gpsDist: " << gps_Dist << ", gpsBear: " << gps_Bear*57.3 << std::endl;
}





//Imu Callback
void imuCallback(const sensor_msgs::Imu::ConstPtr& imuData)
{
	magnet_X = imuData -> orientation.x;
	magnet_Y = imuData -> orientation.y;
	magnet_Z = imuData -> orientation.z;
	magnet_W = imuData -> orientation.w;
	
	gyro_X = imuData -> angular_velocity.x;
	gyro_Y = imuData -> angular_velocity.y;
	gyro_Z = imuData -> angular_velocity.z;
	
	accel_X = imuData -> linear_acceleration.x;
	accel_Y = imuData -> linear_acceleration.y;
	accel_Z = imuData -> linear_acceleration.z;
}



//Imu Process
void imuProcess()
{
	
	tf::Quaternion quatImu(magnet_X, magnet_Y, magnet_Z, magnet_W);
	tf::Matrix3x3 matTf(quatImu);
	matTf.getRPY(roll_Imu, pitch_Imu, yaw_Imu);
	yaw_Imu = yaw_Imu+PI;
	cout<<"Husky Bearing Radians: "<<yaw_Imu<< " Husky Bearing(Deg): "<< (yaw_Imu)*57.3<<endl;
}


//Sonar Callback

//Side Sonar front
void sonar1Callback(const std_msgs::Float32::ConstPtr& sonar_side_front)
{
	snr_Range_1 = sonar_side_front;	
	snr_Range_1 = (snr_Range_1*1000);
	
}

//Side Sonar Middle low
void sonar2Callback(const std_msgs::Float32::ConstPtr& sonar_side_mid_b)
{
	snr_Range_2 = sonar_side_mid_b;	
	snr_Range_2 = (snr_Range_2*1000);
	
}

//Side Sonar back
void sonar3Callback(const std_msgs::Float32::ConstPtr& sonar_side_back)
{
	snr_Range_3 = sonar_side_back ;	
	snr_Range_3 = (snr_Range_3*1000);
	
}

//Side Sonar Middle high
void sonar4Callback(const std_msgs::Float32::ConstPtr& sonar_side_mid_t)
{
	snr_Range_2_high = sonar_side_mid_t;
	snr_Range_2_high = (snr_Range_2_high*1000);
	
}

//Front Sonar Left
void sonar5Callback(const std_msgs::Float32::ConstPtr& sonar_front_l)
{
	snr_Range_front_Left = sonar_front_l;
	snr_Range_front_Left = (snr_Range_front_Left *1000);
	
}

//Front Sonar Right
void sonar6Callback(const std_msgs::Float32::ConstPtr& sonar_front_r)
{
	snr_Range_front_Right = sonar_front_r ;
	snr_Range_front_Right = (snr_Range_front_Right*1000);
	
}


//This function computes the length of the panel from the center of the husky to the right
double Length_Side_Front()
{
double r1(0);
double r2(0);
double length(0);
double theta1(0);
double theta2(0);
int ind(0);
int count = 0;
for(int i=91; i<272; i++)
{
        if((laser_Read[i] > 0.1)&&(laser_Read[i] < max_range_prox)){
	//Reject the first reading because of corner noise        
	if (count == 0){
        	count++;
        }
        else if(count == 1){
		theta1 = (-45+(i*0.5))*(PI/180);
		r1 = laser_Read[i];
		//cout<<"Index: "<<i<<" theta1: "<<theta1<<" Range1: "<<r1<<endl;
		count++;}
	else{
		theta2 = (-45+((i-2)*0.5))*(PI/180);
		r2 = laser_Read[i-2];}
		ind = (i-2);
	}
}
	
double a = r2*sin(theta2)-r1*sin(theta1);
double b = r2*cos(theta2)-r1*cos(theta1);
length = pow(a,2) + pow(b,2);
length = pow(length,0.5);
//cout<<"Index: "<<ind<<" theta2: "<<theta2<<" Range2: "<<r2<<endl;
//cout<<"Length_Side_Front:"<<length<<endl;
return length;	
}


//Funtion to tun left 90degree
void Left_Turn (double start_bear)
{
	double current_bear = yaw_Imu;
	double err = current_bear - start_bear;
	err = atan2(sin(err),cos(err));
	double err_th = 2*deg2rad;
	while (abs(err-0.5*PI)>err_th)
	{
		cmdAng_Z = 0.2;
		current_bear = atan2(sin(yaw_Imu),cos(yaw_Imu));
		err = start_bear - current_bear;
		err = atan2(sin(err),cos(err));
		err = abs(err);
	}


}

void Right_Turn (double start_bear)
{
	double current_bear = yaw_Imu;
	double err = current_bear - start_bear;
	err = atan2(sin(err),cos(err));
	double err_th = 2*deg2rad;
	while (abs(err-0.5*PI)>err_th)
	{
		cmdAng_Z = -0.2;
		current_bear = atan2(sin(yaw_Imu),cos(yaw_Imu));
		err = start_bear - current_bear;
		err = atan2(sin(err),cos(err));
		err = abs(err);
	}
}








/*****************************************
/*********   DRIVE FUNCTIONS   **********
*****************************************/

//Drive based on GPS fcn - Mode = 0
void moveGps()
{
     double Kp_gps = 5;	
    // double minVal = 5;
    // int indMin(0);
     for (int i = 91; i<450; i++)
     {
	if((laser_Read[i] > 0.3)&&(laser_Read[i] < 5))
        {
            mode = 1;

        }
     }     
     if(count_gpspt <3)
     {           
         
            gpsProcess(ptTarget_Long[count_gpspt],ptTarget_Latt[count_gpspt]);
	    imuProcess();
	
	    // Switch to mode 1 once enters the outer neighborhood
	    //if(gps_Dist >= dist_GpsToFind){
	if(gps_Dist >= 2 )
        {
            cmdAng_Z = gps_Bear - yaw_Imu;
	    cmdAng_Z = atan2(sin(cmdAng_Z),cos(cmdAng_Z));
            cout<<"Error in orientation: "<< gps_Bear - yaw_Imu<<cmdAng_Z<<" Num_point:   		  			"<<count_gpspt<<endl;
            if (abs(cmdAng_Z)>3*deg2rad)
            {
                cmdLin_X = 0.2;
		if (cmdAng_Z > 0){
			cmdAng_Z = 0.2;}
		else if (comdAng_Z<0)
		{
			cmdAng_Z = -0.2;
		}
		else
		{
			cmdAng_Z = 0;
		}
              
                
            }
            else
            {
                cmdLin_X = 10*cstSpd_X;
		cmdAng_Z = Kp_gps*cmdAng_Z;
            }
            cout <<"Destination Point Number: "<<count_gpspt<<endl;
        }
	else
        {
            count_gpspt++;
	    cmdLin_X = 0;
            cmdAng_Z = 0;
		    //mode = 1;
            std:cout << "Switching to APPROACH mode" << std::endl;
        }
    }
    else
    {
        cmdLin_X = 0;
	cmdAng_Z = 0;
    }	
}





//Station Find Function - Mode = 1

void findStat()
{
    double minVal = maxDist_Find;
    int indMin(0);
    for(int i=91; i<450; i++){
        if((laser_Read[i] > 0.1)&&(laser_Read[i] < maxDist_Find)){
            if (laser_Read[i] < minVal){
                minVal = laser_Read[i];
                indMin = i;
            }
            
        }
    }
    first_Deg = (-45+(indMin*0.5))*(PI/180);
    omega_Des = first_Deg-(0.5*PI);
    first_LaserNonZero = laser_Read[indMin];
    
    // Switch to mode 2 once enters the inner neighborhood
    
    if(first_LaserNonZero >= dist_FindToRotate){
        cmdLin_X=5*cstSpd_X;
        cmdAng_Z=K_p*omega_Des;
        //Clipping the minimum value
        if (abs(omega_Des*57.32)<3){
            cmdAng_Z = 0;
            
        }
    }
    
    else if ((first_LaserNonZero < dist_FindToRotate)&&(first_LaserNonZero >0.1)){
        mode = 2; // Switch to mode 9 for rotation
        std::cout << "Switching to First Approach mode" << std::endl;}
    
}




//Mode = 2;
void Rotation_newCode()
{
    double Kp_v(0.1);
    double Kp_wa(0.7);
    //double Kp_wd(0.3);
    double minVal = 10;
    double w_max(0.25);	
    int firstIdx = 0;
    int indMin = 0;
    int countFor(0);
    
    double length_side_full(0);
    double length_side_front(0);
    COUNT_Delay++;

//In this for loop, we compute the minimum reading of the LIDAR and 
//its index to compute the angle
    for(int i=50; i<450; i++){
        if(laser_Read[i] > 0.1){
        	if(laser_Read[i] < max_range_prox){
		        if (laser_Read[i] < minVal){
	                minVal = laser_Read[i];
	                indMin = i;
		        }
		       
		    }
        }
    
   // If LIDAR has a valid reading enter the if statement. 
    if (minVal!=10)
    {
    	double tan_theta(0);
    	
    	float dist_12 = 270; // Distance between sonars 1 and 2 in mm
    	float dist_23 = 285; // Distance between sonars 2 and 3 in mm
    	float dist_13 = dist_12 + dist_23;
    	double diff_sonar_12(0);
    	double diff_sonar_13(0);
    	double diff_sonar_23(0);
    	double diff_sonar13_est(0);
    	double err_side(0);
    	double m1(0),m2(0);
	
	// Use SONARS to decide when to stop
	if ((snr_Range_1<max_snr_rng)&&(snr_Range_2<max_snr_rng)&&(snr_Range_3<max_snr_rng)&&(snr_Range_2_high<max_snr_rng)&&(snr_Range_1>0)&&(snr_Range_2>0)&&(snr_Range_3>0)&&(snr_Range_2_high>0)){
		diff_sonar_12 = snr_Range_1 - snr_Range_2;
		diff_sonar_13 = snr_Range_1 - snr_Range_3;
		diff_sonar_23 = snr_Range_2 - snr_Range_3;
		m1 = diff_sonar_23/dist_23;
		m2 = diff_sonar_12/dist_12;
		err_side = m1 - m2;
		cout<<"Sonar 1: "<<snr_Range_1<<" Sonar_2_low: "<<snr_Range_2<<" Sonar_2_high: "<<snr_Range_2_high<<" Sonar3: "<<snr_Range_3<<" Slope 1: "<<m1<<" Slope 2: "<<m2<<" Error: "<<err_side<<endl;
		
		if (abs(err_side)<0.4){
			//cout<<"Parking Mode"<<endl;
			//mode = 3;
			//double temp = (snr_Range_1 + snr_Range_2 + snr_Range_3)/3;
			double temp = snr_Range_2;
			double err_height = abs(temp - snr_Range_2_high);
			cout<<"Sonar bottom - Sonar top: "<< err_height<<endl;
			if( err_height > 180)
			{
				mode = 41;
			}

		}
	}
		
	
	firstIdx = indMin;
    	double first_Deg = (-45+(firstIdx*0.5))*(PI/180);
        
	
	
	//Proportional Controller for Linear velocity
	//If the angle of closest point is less than 30 degs, forward velocity is 0.08;	
	if (abs(first_Deg) > (30*0.0175)) { 
		cmdLin_X = 0.08;
	}
	else{      
		cmdLin_X = Kp_v/(abs(first_Deg)+0.01); // linear velocity as function of angle
        	if (cmdLin_X>4*cstSpd_X){
            		cmdLin_X =4*cstSpd_X;       // Saturating the linear velocity (cstSpd = 0.05)
        	}
	}
	
	
	// Prop. controller for angular velocity
        //double dist_err = Kp_wd*minVal;
        
        if (count_mode2 == 0)
        {
        	err_w_mode2 = first_Deg;
        	count_mode2++;
        }
        else
        {
        	err_w_mode2 = (err_w_mode2+first_Deg)/2;
        	//err_w_mode2 = first_Deg;
        }
        cmdAng_Z=Kp_wa*(err_w_mode2); 
        if (abs(cmdAng_Z)>w_max){
            if (cmdAng_Z > 0){
                cmdAng_Z = w_max;}
            else{
                cmdAng_Z = -w_max;}
        }
        
	//Clipping the minimum value
	if (abs(err_w_mode2*57.32)<3){
            cmdAng_Z = 0;
            
        }
    }
	
    else if((minVal==10)&&(COUNT_Delay>100)){
    mode = 21;}
    else{
    	mode = 2;
    	
    }
    
   
}

//Mode = 21: Open loop turn at the corners
void Rotation_newCode_Corner()
{
    int count(0);
    cmdAng_Z= -0.2;
    cmdLin_X= 0.01;
// Quit the open loop turn right if the LIDAR is detecting more than 15 points from 20 to 100	
    for(int i=20; i<100; i++){
        if(laser_Read[i] > 0.1 && laser_Read[i]<max_range_prox){
            count= count+1;
        }
    }
	
    if (count>30){ 
        mode = 2;
    }
}



//Mode 41
void Front_ParkingBack(){
	cmdAng_Z = -0.25;
	cmdLin_X =-0.08;
	if (count_mode41<10)
	{
		cmdAng_Z = 0;
		cmdLin_X = 0.1;
		count_mode41++;
	}
	cout << "Sonar_front_Left: " << snr_Range_front_Left << endl;
	cout << "Sonar_front_Right: " << snr_Range_front_Right << endl;
	float minVal = 10;
	int indMin(0);
	for(int i=50; i<450; i++){
        if((laser_Read[i] > 0.1)&&(laser_Read[i] < max_range_prox)){
        	if (laser_Read[i] < minVal){
	                minVal = laser_Read[i];
	                indMin = i;
		}
	    }       
	}
	int err_align = 270 - indMin;
	if ((snr_Range_front_Left<max_snr_rng)&&(snr_Range_front_Right<max_snr_rng)&&(abs(err_align)<30)){
	mode = 42;	} 
}

//Align Sonar Function - Mode = 42
void alignSonar()
{
    cmdLin_X = 0;
    float Kp_w = 0.008;
    float err_align = snr_Range_front_Right - snr_Range_front_Left;
    if (abs(err_align)>2){
    	cmdAng_Z = Kp_w*err_align;
	cmdLin_X = 0.01;
    	
    }
    else
    {
    	cmdAng_Z =0;
	cmdLin_X = 0;
    	mode = 43; 
    }
    
    cout << "sonarDiff: " << abs(snr_Range_front_Right - snr_Range_front_Left) << endl;
}

//Mode= 43 (Front Parking1)
/*
void Front_ParkingFinal()
{
    
    double min_dist = min(snr_Range_front_Left,snr_Range_front_Right);
    if(min_dist >950){
    	cmdLin_X = 0.1;}
    else if(min_dist <850)
    {
	cmdLin_X =  -0.08;}
    else{
    	cmdLin_X = 0;
    	mode = 10;
    }
    cmdAng_Z =0;	

    float Kp_w = 0.008;
    float err_align = snr_Range_front_Right - snr_Range_front_Left;
    if (abs(err_align)>2){
    	cmdAng_Z = Kp_w*err_align;
    	
    }
    else
    {
    	cmdAng_Z = 0;
	cmdLin_X = 0;
	mode = 10;
    }

     cout << "sonarDiff: " << abs(snr_Range_front_Right - snr_Range_front_Left) << endl;
}
*/
//Mode= 43 (Front Parking1)

void Front_ParkingFinal()
{
	
   double minVal(10);
   int indMin(0);
   double err(0),Kp_w(0.5);
   int cone_size = 10;
   int step_size = 1;
   double err_align(0);
   //In this for loop, we compute the minimum reading of the LIDAR and 
//its index to compute the angle
    for(int i=0; i<450; i++){
        if(laser_Read[i] > 0.1){
        	if(laser_Read[i] < max_range_prox){
		        if (laser_Read[i] < minVal){
	                minVal = laser_Read[i];
	                indMin = i;
	                //cout<<"Minimum Distance: "<< minVal<<"  Minimum Index: "<< indMin<<endl;
		        }
		       
		    }
        }
    } 
    
    if(minVal >1.05)
    {
    	cmdLin_X = 0.1;
    }
    else if (minVal < 0.95)
    {
	cmdLin_X = -0.1;
    }
    else
    {
    	cmdLin_X = 0;
    	mode = 10;
    }
    cmdAng_Z =0;
   	
    
    
}

/*****************************************
/*****************************************
/***********   MAIN FUNCTION   ***********
/*****************************************
*****************************************/
int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "driveRover");
	ros::NodeHandle nodeDrvLsr;
	ros::Rate loop_rate(10);
	
	ros::Subscriber subScan = nodeDrvLsr.subscribe<sensor_msgs::LaserScan>("scan", 100, scanCallback);
	ros::Subscriber subGps = nodeDrvLsr.subscribe<sensor_msgs::NavSatFix>("gps/fix", 5, gpsCallback);
	ros::Subscriber subImu = nodeDrvLsr.subscribe<sensor_msgs::Imu>("imu/data", 50, imuCallback);
	ros::Subscriber subSonar1 = nodeDrvLsr.subscribe<std_msgs::Float32>("sonar_side_front", 10, sonar1Callback);
	ros::Subscriber subSonar2 = nodeDrvLsr.subscribe<std_msgs::Float32>("sonar_side_mid_b", 10, sonar2Callback);
	ros::Subscriber subSonar3 = nodeDrvLsr.subscribe<std_msgs::Float32>("sonar_side_back", 10, sonar3Callback);
	ros::Subscriber subSonar4 = nodeDrvLsr.subscribe<std_msgs::Float32>("sonar_side_mid_t", 10, sonar4Callback);
	ros::Subscriber subSonar5 = nodeDrvLsr.subscribe<std_msgs::Float32>("sonar_front_l", 10, sonar5Callback);
	ros::Subscriber subSonar6 = nodeDrvLsr.subscribe<std_msgs::Float32>("sonar_front_r", 10, sonar6Callback);
	
	//Advertise velocity command publisher
    	ros::Publisher commandPub = nodeDrvLsr.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 1);
    	
    	//Mode publisher to SM
    	ros::Publisher mode_Pub = nodeDrvLsr.advertise<std_msgs::Int32>("mode_h_main", 100);
	
	//Main loop
	while (ros::ok())
	{
		cout<<"Mode: "<<mode<<endl;
		laser_Read = laser_Read_real;
		num_laser = 0;
		if(mode == 0){moveGps();
			std::cout << "moveGps" << std::endl;} 
		else if(mode == 1){findStat();
			std::cout << "findStat" << std::endl;}
		
        	else if(mode == 2){Rotation_newCode();
            		std::cout << "Rotation_newCode" << std::endl;}
        	else if(mode == 21){Rotation_newCode_Corner();
            		std::cout << "Rotation_newCode_Corner" << std::endl;}
		else if(mode == 41){Front_ParkingBack();
            		std::cout << "Front_ParkingBack" << std::endl;}
        	else if(mode == 42){alignSonar();
            		std::cout << "alignSonar" << std::endl;}
        	else if(mode == 43){Front_ParkingFinal();
            		std::cout << "Front_ParkingFinal" << std::endl;}
		else if(mode == 9){ imuProcess();
            		std::cout << "ImuProcess" << std::endl;}

        	std::cout << "Speed: X [" << cmdLin_X << "], Y [" << cmdLin_Y << "], Z [" << cmdAng_Z << "]" << std::endl;        
            	
		// Saturate cmdLin_X and cmdAng_Z
		if(cmdLin_X >= satLin_X){cmdLin_X = satLin_X;}
		else if(cmdLin_X <= -satLin_X){cmdLin_X = -satLin_X;}
		if(cmdAng_Z >= satAng_Z){cmdAng_Z = satAng_Z;}
		else if(cmdAng_Z <= -satAng_Z){cmdAng_Z = -satAng_Z;}
		vel_Cmd.linear.x=cmdLin_X;
		vel_Cmd.linear.y=0;
		vel_Cmd.linear.z=0;
		vel_Cmd.angular.x=0;
		vel_Cmd.angular.y=0;
		vel_Cmd.angular.z=cmdAng_Z;
		commandPub.publish(vel_Cmd);
		
		//Send the current mode to the SM
		mode_SM.data = mode;
		mode_Pub.publish(mode_SM);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
