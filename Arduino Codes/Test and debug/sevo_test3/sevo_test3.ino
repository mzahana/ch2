#include <Servo.h>

Servo Servo1;
Servo Servo2;
int servoPin1 = 25;
int servoPin2 = 29;

int min_pos = 900;
int max_pos = 2100;

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  pinMode (servoPin1,OUTPUT);
  pinMode (servoPin2,OUTPUT);
  Servo1.attach(servoPin1);
  Servo1.writeMicroseconds(max_pos);
  Servo2.attach(servoPin2);
  Servo2.writeMicroseconds(max_pos);
  //delay(10000);
  int obs = analogRead(A0);
  Serial.println(obs);
}

void loop() {
//  
//   int feedback = analogRead(A0);
//   Serial.println(feedback);
//   Servo2.writeMicroseconds(900);
   
}
