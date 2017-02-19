#include <Servo.h>

Servo Servo1;
Servo Servo2;
int servoPin1 = 25;
int servoPin2 = 29;
int min_pos = 1250;
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
  Servo1.attach(servoPin1);
  Servo1.writeMicroseconds(max_pos);
  Servo2.attach(servoPin2);
  Servo2.writeMicroseconds(max_pos);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  obs[0] = analogRead(A0);
  Serial.println(obs[0]);
}

void loop() {
  
  for (int i=max_pos; i>=min_pos; i -= 10){
      Servo1.writeMicroseconds(i);
      
        // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(A0);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  Serial.println(average);
  obs[1] = average;
//  if ((abs(obs[1]-obs[0])<1 && i<2000)){
//        Servo2.writeMicroseconds(max_pos);
//         break;        
//   }     
  obs[0] = obs[1];
  delay(300); 
  // delay in between reads for stability
  }
  Servo1.writeMicroseconds(max_pos);
  delay(8000);
}
