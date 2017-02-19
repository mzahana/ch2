/* Ping sensor ROS code */

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  pp;
std_msgs::Bool pushed_msg1;
std_msgs::Bool pushed_msg2;
std_msgs::Bool pushed_msg3;
std_msgs::Bool pushed_msg4;

ros::Publisher pub_button1("switch1",&pushed_msg1);
ros::Publisher pub_button2("switch2",&pushed_msg2);
ros::Publisher pub_button3("switch3",&pushed_msg3);
ros::Publisher pub_button4("switch4",&pushed_msg4);


const int button_pin1 = 3;
const int button_pin2 = 4;
const int button_pin3 = 5;
const int button_pin4 = 6;
unsigned long range_timer;

boolean last_reading1;
boolean last_reading2;
boolean last_reading3;
boolean last_reading4;
long last_debounce_time1=0;
long last_debounce_time2=0;
long last_debounce_time3=0;
long last_debounce_time4=0;
long debounce_delay=50;
boolean published1 = false;
boolean published2 = false;
boolean published3 = false;
boolean published4 = false;


void setup(){
  pp.getHardware()->setBaud(57600);
  pp.initNode();
  pp.advertise(pub_button1);
  pp.advertise(pub_button2);
  pp.advertise(pub_button3);
  pp.advertise(pub_button4);
  
  pinMode(button_pin1, INPUT);
  pinMode(button_pin2, INPUT);
  pinMode(button_pin3, INPUT);
  pinMode(button_pin4, INPUT);
  
  last_reading1 = digitalRead(button_pin1);
  last_reading2 = digitalRead(button_pin2);
  last_reading3 = digitalRead(button_pin3);
  last_reading4 = digitalRead(button_pin4);
}

void loop(){

  boolean reading1 = digitalRead(button_pin1);
  boolean reading2 = digitalRead(button_pin2);
  boolean reading3 = digitalRead(button_pin3);
  boolean reading4 = digitalRead(button_pin4);
  
  if ( (millis()-range_timer) > 20){
    pushed_msg1.data = reading1;
    pub_button1.publish(&pushed_msg1);
    pushed_msg2.data = reading2;
    pub_button2.publish(&pushed_msg2);
    pushed_msg3.data = reading3;
    pub_button3.publish(&pushed_msg3);
    pushed_msg4.data = reading4;
    pub_button4.publish(&pushed_msg4);
     range_timer = millis();
  }

  pp.spinOnce();
}


