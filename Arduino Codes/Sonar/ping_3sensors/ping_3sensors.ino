/* Ping sensor ROS code */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  pingsens;
sensor_msgs::Range range_msg1;
sensor_msgs::Range range_msg2;
sensor_msgs::Range range_msg3;
ros::Publisher pub_range1( "sonar1", &range_msg1);
ros::Publisher pub_range2( "sonar2", &range_msg2);
ros::Publisher pub_range3( "sonar3", &range_msg3);

const int pingPin1 = 6;
const int pingPin2 = 7;
const int pingPin3 = 5;

unsigned long range_timer;

float getRange1(int pingPin1){
 
    float duration, cm, m;
    pinMode(pingPin1, OUTPUT);
    digitalWrite(pingPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin1, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin1, LOW);
    
    pinMode(pingPin1, INPUT);
    duration = pulseIn(pingPin1, HIGH);
    cm = duration / 29 / 2;
    m = cm / 100;

    return m;
}


float getRange2(int pingPin2){
 
    float duration, cm, m;
    pinMode(pingPin2, OUTPUT);
    digitalWrite(pingPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin2, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin2, LOW);
    
    pinMode(pingPin2, INPUT);
    duration = pulseIn(pingPin2, HIGH);
    cm = duration / 29 / 2;
    m = cm / 100;

    return m;
}

float getRange3(int pingPin3){
 
    float duration, cm, m;
    pinMode(pingPin3, OUTPUT);
    digitalWrite(pingPin3, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin3, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin3, LOW);
    
    pinMode(pingPin3, INPUT);
    duration = pulseIn(pingPin3, HIGH);
    cm = duration / 29 / 2;
    m = cm / 100;

    return m;
}

char frameid1[] = "/ping_ranger1";
char frameid2[] = "/ping_ranger2";
char frameid3[] = "/ping_ranger3";

void setup(){
  pingsens.getHardware()->setBaud(57600);
  pingsens.initNode();
  pingsens.advertise(pub_range1);
  pingsens.advertise(pub_range2);
  pingsens.advertise(pub_range3);
  
  range_msg1.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg1.header.frame_id = frameid1;
  range_msg1.field_of_view = 0.25;
  range_msg1.min_range = 0.02;
  range_msg1.max_range = 2;

  range_msg2.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg2.header.frame_id = frameid2;
  range_msg2.field_of_view = 0.25;
  range_msg2.min_range = 0.02;
  range_msg2.max_range = 2;
  
  range_msg3.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg3.header.frame_id = frameid3;
  range_msg3.field_of_view = 0.25;
  range_msg3.min_range = 0.02;
  range_msg3.max_range = 2;
}

void loop(){

  if ( (millis()-range_timer) > 50){
    range_msg1.range = getRange1(pingPin1);
    range_msg2.range = getRange2(pingPin2);
    range_msg3.range = getRange3(pingPin3);
    range_msg1.header.stamp = pingsens.now();
    range_msg2.header.stamp = pingsens.now();
    range_msg3.header.stamp = pingsens.now();
    pub_range1.publish(&range_msg1);
    pub_range2.publish(&range_msg2);
    pub_range3.publish(&range_msg3);
    range_timer = millis();
  }
  pingsens.spinOnce();
}


