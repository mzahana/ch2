#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  grip;
std_msgs::Bool pushed_msg1;
std_msgs::Bool pushed_msg2;
std_msgs::Bool pushed_msg3;
std_msgs::Bool pushed_msg4;

std_msgs::UInt16 current_mode;

std_msgs::UInt16 check_data;

std_msgs::UInt16 closure_status;


ros::Publisher pub_button1("switch1",&pushed_msg1);
ros::Publisher pub_button2("switch2",&pushed_msg2);
ros::Publisher pub_button3("switch3",&pushed_msg3);
ros::Publisher pub_button4("switch4",&pushed_msg4);

ros::Publisher curr_mode("Current_gripper_mode",&current_mode);
ros::Publisher check_variable("ckgripper",&check_data);

ros::Publisher key_grip_status("key_gripped",&closure_status);

const int button_pin1 = 7;
const int button_pin2 = 6;
const int button_pin3 = 5;
const int button_pin4 = 4;
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
boolean published1 = true;
boolean published2 = true;
boolean published3 = true;
boolean published4 = true;

int mode = 0;

int gripper_mode_data;
int gripper_mode_termination_data;

int open_close_gripper;

//////////////////////////////////////////

void gripper_cb( const std_msgs::UInt16& cmd_msg){
     gripper_mode_data = cmd_msg.data;
}

ros::Subscriber<std_msgs::UInt16> sub("gripper_mode", gripper_cb);


void gripper_term_cb( const std_msgs::UInt16& cmd_msg2){
     gripper_mode_termination_data = cmd_msg2.data;
}

ros::Subscriber<std_msgs::UInt16> sub2("gripper_mode_termination", gripper_term_cb);



void key_grip_cb( const std_msgs::UInt16& cmd_msg3){
     open_close_gripper = cmd_msg3.data;
}

ros::Subscriber<std_msgs::UInt16> sub3("open_close_gripper", key_grip_cb);

///////////////////////////////////////

Servo Servo_front;
Servo Servo_back;
int servoPin1 = 25;
int servoPin2 = 29;
int min_pos = 900;
int max_pos = 2100;
int const m = 2;
int obs[m];
const int numReadings = 10;
int readings[numReadings];    
int readIndex = 0;
int total = 0;                 
int average = 0;

///////////////////////////////////////

void setup(){
  analogReference(EXTERNAL);
  pinMode (servoPin1,OUTPUT);
  pinMode (servoPin2,OUTPUT);
  Servo_front.attach(servoPin1);
  Servo_front.writeMicroseconds(max_pos);
  Servo_back.attach(servoPin2);
  Servo_back.writeMicroseconds(max_pos);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  obs[0] = analogRead(A2);
  
  grip.getHardware()->setBaud(57600);
  grip.initNode();
  grip.advertise(pub_button1);
  grip.advertise(pub_button2);
  grip.advertise(pub_button3);
  grip.advertise(pub_button4);
  grip.advertise(curr_mode);
  grip.advertise(check_variable);
  grip.advertise(key_grip_status);
  grip.subscribe(sub);
  grip.subscribe(sub2);
  grip.subscribe(sub3);
  
  pinMode(button_pin1, INPUT_PULLUP);
  pinMode(button_pin2, INPUT_PULLUP);
  pinMode(button_pin3, INPUT_PULLUP);
  pinMode(button_pin4, INPUT_PULLUP);
  
  last_reading1 = digitalRead(button_pin1);
  last_reading2 = digitalRead(button_pin2);
  last_reading3 = digitalRead(button_pin3);
  last_reading4 = digitalRead(button_pin4);
}

void loop(){

  check_data.data = gripper_mode_data;
  check_variable.publish(&check_data);
  
  if(gripper_mode_data == 1){
      mode = 1;
      current_mode.data = mode;
      curr_mode.publish(&current_mode);
      panel_allign();
  }    
  else if(gripper_mode_data == 2){   
      mode = 2;
      current_mode.data = mode;
      curr_mode.publish(&current_mode);
      valve_pinch();
  }
  else if(gripper_mode_data == 3){
     mode = 3;
     current_mode.data = mode;
     curr_mode.publish(&current_mode);
     key_grip();
  }
 

  grip.spinOnce();
}


void panel_allign(){
  
  
  while(gripper_mode_termination_data!=1){
    boolean reading1 = digitalRead(button_pin1);
    boolean reading2 = digitalRead(button_pin2);
    boolean reading3 = digitalRead(button_pin3);
    boolean reading4 = digitalRead(button_pin4);
    if(reading1== true){
       reading1 = false;
    }
    else if(reading1 == false){
      reading1 = true;
    }
  
    if(reading2== true){
       reading2 = false;
    }
    else if(reading2 == false){
      reading2 = true;
    }
  
    if(reading3== true){
       reading3 = false;
    }
    else if(reading3 == false){
      reading3 = true;
    }
  
    if(reading4== true){
       reading4 = false;
    }
    else if(reading4 == false){
      reading4 = true;
    }

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
    grip.spinOnce();
  }    
    
}


void valve_pinch(){
}



void key_grip(){
  while(gripper_mode_termination_data!=1){ 
    if(open_close_gripper == 1){
        close_grip();
    }
    else if(open_close_gripper == 2){
        open_grip();
    }
  }
  open_grip();
}

void close_grip(){
  
   for (int j=1; j<=16;j++){ 
    Servo_front.writeMicroseconds(min_pos);
    delay(50);
    grip.spinOnce();
   }
   
    for (int i=max_pos; i>=min_pos; i -= 10){
  
      Servo_back.writeMicroseconds(i);
      delay(100);
      total = total - readings[readIndex];
      readings[readIndex] = analogRead(A2);
      total = total + readings[readIndex];
      readIndex = readIndex + 1;
      if (readIndex >= numReadings) {
      readIndex = 0;
      }
      average = total / numReadings;
  
      //Serial.println(average);
      delay(100);
      obs[1] = average;
      if ((abs(obs[1]-obs[0])<1 && i<1800)||average>725){
        Servo_back.writeMicroseconds(i+100);
         break;        
      }     
      obs[0] = obs[1];           
  }
      while(gripper_mode_termination_data!=1){
          delay(50);
          grip.spinOnce();
      }
}


void open_grip(){
  Servo_back.writeMicroseconds(max_pos);
  for (int i=1; i<=16; i++){
    delay(50);
    grip.spinOnce();
    Servo_front.writeMicroseconds(max_pos);
  }
}
