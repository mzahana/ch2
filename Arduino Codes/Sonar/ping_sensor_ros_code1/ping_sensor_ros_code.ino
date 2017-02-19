/* Ping sensor ROS code */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  pingsens;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "MyRange", &range_msg);


const int pingPin = 7;
unsigned long range_timer;

float getRange(int pingPin){
 
    float duration, cm, m;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
    
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
    cm = duration / 29 / 2;
    m = cm / 100;

    return m;
}

char frameid[] = "/ping_ranger";

void setup(){

  pingsens.initNode();
  pingsens.advertise(pub_range);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = frameid;
  range_msg.field_of_view = 0.25;
  range_msg.min_range = 0.02;
  range_msg.max_range = 2;
  
}

void loop(){

  if ( (millis()-range_timer) > 50){
    range_msg.range = getRange(pingPin);
    range_msg.header.stamp = pingsens.now();
    pub_range.publish(&range_msg);
    range_timer = millis();
  }
  pingsens.spinOnce();
}


