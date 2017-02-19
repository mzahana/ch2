#include <Servo.h>

Servo Servo1;                //declair the servo!
int reading[20];
int Feedback[400];
int servoPin1 = 6;
int test;               //general purpose int
int offset = 0;
int noise = 50;
int mean;
int result;
int min_pos = 900;
int max_pos = 2100;
boolean done;
int k = 0;


void setup() {
  Serial.begin(9600);                     // initialize serial output
  analogReference(EXTERNAL);
  pinMode (servoPin1,OUTPUT);
  Servo1.attach(servoPin1);
  analogReference(DEFAULT); 
  // turn on servo control at digital pin 2
  setRange();                             // go test the range and set the values
}

void loop() {
   
  Servo1.writeMicroseconds(min_pos);
  delay(2000);
  for (int i=min_pos; i<=max_pos; i+=10){
    Servo1.writeMicroseconds(i);
    delay (50);
    test = getFeedback();
//    offset = test - Feedback[i];
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(test*(5.0/1023.0));
//    Serial.print("  ");
//    Serial.print(offset);
//    Serial.print("  ");
//    Serial.println(Feedback[i]);
  }
  delay (10000);
 Serial.println(" ");
}



void setRange(){

  Servo1.writeMicroseconds(min_pos);                  // send servo to 0 degree position
  delay(20000);                      // give servo enough time to get there
  for (int i=min_pos; i<=max_pos; i += 3){
    Servo1.writeMicroseconds(i);                // send next degree pulse to servo
    delay(100);                      // let things settle down
    Feedback[k] = getFeedback();    // read the servo feedback
    Serial.print(k);
    Serial.print(" = ");
    Serial.println(Feedback[k]*(5.0/1023.0));
    k++;
  }
}


int getFeedback(){

   for (int j=0; j<20; j++){
      reading[j] = analogRead(0);    //get raw data from servo potentiometer
      delay(3);
    }                                // sort the readings low to high in array                                
    done = false;                    // clear sorting flag             
    while(done != true){             // simple swap sorts numbers from lowest to highest
    done = true;
    for (int j=0; j<20; j++){
      if (reading[j] > reading[j + 1]){     // sorting numbers here
        test = reading[j + 1];
        reading [j+1] = reading[j] ;
        reading[j] = test;
        done = false;
       }
     }
   }
//  for (int j=0; j<20; j++){        //un-comment this for-loop to see the raw ordered data
//      Serial.print(i);
//      Serial.print("  ");
//      Serial.println(reading[j]);
//  }
    mean = 0;
    for (int k=6; k<14; k++){        //discard the 6 highest and 6 lowest readings
      mean += reading[k];
    }
    result = mean/8;              //average useful readings
    return (result);
}
// END OF SERVO_WITH_FEEDBACK_V1
