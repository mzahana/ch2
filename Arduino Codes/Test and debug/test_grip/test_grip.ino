#include <Servo.h>

Servo Servo1;
Servo Servo2;
int servoPin1 = 25;
int servoPin2 = 29;
int min_pos = 900;
int max_pos = 2100;
float feedback;
float total_feedback;
int const m = 2;
int obs[m];
int obs_der[m];
int obs_der_max = 0;
int k = 0;
float max_dum = 0;
float min_dum = 0;
int resume_ops = 0;
int smooth_reading[10];

const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  pinMode (servoPin1,OUTPUT);
  pinMode (servoPin2,OUTPUT);
  Servo1.attach(servoPin1);
  Servo1.writeMicroseconds(max_pos);
  Servo2.attach(servoPin2);
  Servo2.writeMicroseconds(max_pos);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  obs[0] = analogRead(A2);
  Serial.println(obs[0]);
}

void loop() {
  
  int inByte = Serial.read();

    switch (inByte) {

    case '1':    
      close_grip();
      break;

    case '2':    
      open_grip();
      break;
      
    }  

} 

void close_grip(){
  
    Servo1.writeMicroseconds(min_pos);
    delay(8000);
    
    for (int i=max_pos; i>=min_pos; i -= 10){
  
      Servo2.writeMicroseconds(i);
      delay(100);
      total = total - readings[readIndex];
      readings[readIndex] = analogRead(A2);
      total = total + readings[readIndex];
      readIndex = readIndex + 1;
      if (readIndex >= numReadings) {
      readIndex = 0;
      }
      average = total / numReadings;
  
      Serial.println(average);
      delay(100);
      obs[1] = average;
      if ((abs(obs[1]-obs[0])<1 && i<1800)||average>725){
        Servo2.writeMicroseconds(i+100);
         break;
         
      }
      
      obs[0] = obs[1];
           
  }
      delay(5000);
}


void open_grip(){
  Servo2.writeMicroseconds(max_pos);
  delay(8000);
  Servo1.writeMicroseconds(max_pos);
}
