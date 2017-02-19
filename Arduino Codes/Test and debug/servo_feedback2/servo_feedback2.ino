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
  Servo1.attach(servoPin1);
  Servo1.writeMicroseconds(max_pos);
  Servo2.attach(servoPin2);
  Servo2.writeMicroseconds(max_pos);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  obs[0] = analogRead(A1);
  Serial.println(obs[0]);
}

void loop() {
  
  for (int i=max_pos; i>=min_pos; i -= 5){
      Servo2.writeMicroseconds(i);
//      for (int j=0; j<10; j++){
//        smooth_reading[j]= analogRead(0);
//         delay(10);
//      }
//     
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(A1);
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
  delay(100);        // delay in between reads for stability
  
  obs[1] = average;
  
//  if (abs(obs[1]-obs[0])<1 && i<1900){
//    break;
//  }
  
  obs[0] = obs[1];
      //feedback = analogRead(0);
//      Serial.println(feedback);
//      int v = aver;
//       if (k==m){
//        k = 0;
//      }
//      
//      if(k<m){
//         obs[k] = v;
//         k++;
//      }
//      else{
//        k=0;
//        for (int j=0; j<m; j++){
//          obs_der[j] = obs[j]-obs[j+1];
 //       }
//        for (int l=0; l<m; l++){
//          obs_der_max = max(obs_der[l],obs_der_max);
//        }
//      }
//      if (obs_der_max<20){
//          Servo1.writeMicroseconds(i);
//          obs_der_max = 0;
//          break;
//        }
      
  }
//Serial.println("### Program paused, Enter 1 to resume ###");
//  
//  while(resume_ops!=1){
//    //Serial.println("### Program paused, Enter 1 to resume ###");
//    if (Serial.available() > 0) { 
//       resume_ops = Serial.read();
//    }
//  }
  delay (10000);
  Servo1.writeMicroseconds(max_pos);
  delay (10000);

} 

//void stop_ser(){
//  Servo1.writeMicroseconds(i);
//  
//}
