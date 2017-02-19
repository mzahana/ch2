#include <Servo.h>

Servo servo_front;
Servo servo_back;
// create servo object to control a servo
// twelve servo objects can be created on most boards
int feedback_front;
int feedback_back;

int min_pos = 900;
int max_pos = 2100;
int pos;
int pos_front = min_pos;    // variable to store the servo position
int pos_back = min_pos;

void setup() {
  Serial.begin(9600);      
  analogReference(EXTERNAL);
  servo_front.attach(6);
  servo_back.attach(7);
  servo_front.writeMicroseconds(min_pos);
  servo_back.writeMicroseconds(min_pos);
}

void loop() {
  for (pos = min_pos; pos <= max_pos; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo_front.writeMicroseconds(pos);
    servo_back.writeMicroseconds(pos);
    // tell servo to go to position in variable 'pos'
    delay(50);
    feedback_front = analogRead(0);
    Serial.println(feedback_front);
    // waits 15ms for the servo to reach the position
  }
  delay(20000); 
  servo_front.writeMicroseconds(min_pos);
  servo_back.writeMicroseconds(min_pos);
  delay(20000);  
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
}
