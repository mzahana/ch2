#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <Wire.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>


#define MYAHRS_I2C_ADDRESS           0x20
#define MYMOTION_WHO_AM_I_VALUE      0xB1


///////////////////////////////////////

enum {
    I2C_SLAVE_REG_WHO_AM_I = 0x01,
    I2C_SLAVE_REG_REV_ID_MAJOR,
    I2C_SLAVE_REG_REV_ID_MINOR,
    I2C_SLAVE_REG_STATUS,

    // RAW DATA
    I2C_SLAVE_REG_I_ACC_X_LOW = 0x10,
    I2C_SLAVE_REG_I_ACC_X_HIGH,
    I2C_SLAVE_REG_I_ACC_Y_LOW,
    I2C_SLAVE_REG_I_ACC_Y_HIGH,
    I2C_SLAVE_REG_I_ACC_Z_LOW,
    I2C_SLAVE_REG_I_ACC_Z_HIGH,
    I2C_SLAVE_REG_I_GYRO_X_LOW,
    I2C_SLAVE_REG_I_GYRO_X_HIGH,
    I2C_SLAVE_REG_I_GYRO_Y_LOW,
    I2C_SLAVE_REG_I_GYRO_Y_HIGH,
    I2C_SLAVE_REG_I_GYRO_Z_LOW,
    I2C_SLAVE_REG_I_GYRO_Z_HIGH,
    I2C_SLAVE_REG_I_MAGNET_X_LOW,
    I2C_SLAVE_REG_I_MAGNET_X_HIGH,
    I2C_SLAVE_REG_I_MAGNET_Y_LOW,
    I2C_SLAVE_REG_I_MAGNET_Y_HIGH,
    I2C_SLAVE_REG_I_MAGNET_Z_LOW,
    I2C_SLAVE_REG_I_MAGNET_Z_HIGH,

    // COMPENSATED DATA
    I2C_SLAVE_REG_C_ACC_X_LOW,
    I2C_SLAVE_REG_C_ACC_X_HIGH,
    I2C_SLAVE_REG_C_ACC_Y_LOW,
    I2C_SLAVE_REG_C_ACC_Y_HIGH,
    I2C_SLAVE_REG_C_ACC_Z_LOW,
    I2C_SLAVE_REG_C_ACC_Z_HIGH,
    I2C_SLAVE_REG_C_GYRO_X_LOW,
    I2C_SLAVE_REG_C_GYRO_X_HIGH,
    I2C_SLAVE_REG_C_GYRO_Y_LOW,
    I2C_SLAVE_REG_C_GYRO_Y_HIGH,
    I2C_SLAVE_REG_C_GYRO_Z_LOW,
    I2C_SLAVE_REG_C_GYRO_Z_HIGH,
    I2C_SLAVE_REG_C_MAGNET_X_LOW,
    I2C_SLAVE_REG_C_MAGNET_X_HIGH,
    I2C_SLAVE_REG_C_MAGNET_Y_LOW,
    I2C_SLAVE_REG_C_MAGNET_Y_HIGH,
    I2C_SLAVE_REG_C_MAGNET_Z_LOW,
    I2C_SLAVE_REG_C_MAGNET_Z_HIGH,
    I2C_SLAVE_REG_C_TEMPERATURE_LOW,
    I2C_SLAVE_REG_C_TEMPERATURE_HIGH,

    // Attitude - Euler angle
    I2C_SLAVE_REG_ROLL_LOW,
    I2C_SLAVE_REG_ROLL_HIGH,
    I2C_SLAVE_REG_PITCH_LOW,
    I2C_SLAVE_REG_PITCH_HIGH,
    I2C_SLAVE_REG_YAW_LOW,
    I2C_SLAVE_REG_YAW_HIGH,
    
    //Attitude - Quaternion
    I2C_SLAVE_REG_QUATERNIAN_X_LOW,
    I2C_SLAVE_REG_QUATERNIAN_X_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_Y_LOW,
    I2C_SLAVE_REG_QUATERNIAN_Y_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_Z_LOW,
    I2C_SLAVE_REG_QUATERNIAN_Z_HIGH,
    I2C_SLAVE_REG_QUATERNIAN_W_LOW,
    I2C_SLAVE_REG_QUATERNIAN_W_HIGH,
};

ros::NodeHandle  grip;
std_msgs::Bool pushed_msg1;
std_msgs::Bool pushed_msg2;
std_msgs::Bool pushed_msg3;
std_msgs::Bool pushed_msg4;

geometry_msgs::Vector3 imu_data;

std_msgs::Int32 current_mode;

ros::Publisher pub_button1("switch1",&pushed_msg1);
ros::Publisher pub_button2("switch2",&pushed_msg2);
ros::Publisher pub_button3("switch3",&pushed_msg3);
ros::Publisher pub_button4("switch4",&pushed_msg4);

ros::Publisher check_mode("mode_g_main",&current_mode);
ros::Publisher RPY("imu_gripper",&imu_data);

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
int pinch_again = 0;
int pinch_again_local = 0;

int gripper_mode_data = 0;

//////////////////////////////////////////

void gripper_cb(const std_msgs::Int32& cmd_msg){
     gripper_mode_data = cmd_msg.data;
}

ros::Subscriber<std_msgs::Int32> sub("mode_system_grip", gripper_cb);

///////////////////////////////////////////

void pinch_cb(const std_msgs::Int32& cmd_msg2){
     pinch_again = cmd_msg2.data;
}

ros::Subscriber<std_msgs::Int32> sub2("pinch", pinch_cb);

///////////////////////////////////////

Servo Servo_front;
Servo Servo_back;
int servoPin1 = 25;
int servoPin2 = 29;
int min_pos_back = 1350;
int min_pos_front = 1250;
int max_pos = 2100;
int const m = 2;
int obs[m];
const int numReadings = 10;
int readings[numReadings];    
int readIndex = 0;
int total = 0;                 
int average = 0;


//////////////////////////////////////

/////////////////////// IMU /////////////////////////////


//////////////////////////////////////

void setup(){
  Wire.begin();
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
  grip.advertise(check_mode);
  grip.advertise(RPY);
  grip.subscribe(sub);
  grip.subscribe(sub2);
  grip.spinOnce();
  delay(1000);
  pinMode(button_pin1, INPUT_PULLUP);
  pinMode(button_pin2, INPUT_PULLUP);
  pinMode(button_pin3, INPUT_PULLUP);
  pinMode(button_pin4, INPUT_PULLUP);
  
  last_reading1 = digitalRead(button_pin1);
  last_reading2 = digitalRead(button_pin2);
  last_reading3 = digitalRead(button_pin3);
  last_reading4 = digitalRead(button_pin4);
  
  current_mode.data = mode;
  check_mode.publish(&current_mode);
  
  who_am_i();
  read_rev_id();
  delay(1000);
}

////////////////////////////////////////////////

bool sensor_init()
{
  uint8_t buf_whomi[1];
  uint8_t buf_stat[1];
    
  if(read(I2C_SLAVE_REG_WHO_AM_I, buf_whomi, 1) != 0xB1) {
    return false;
  }
  if(read(I2C_SLAVE_REG_STATUS, buf_stat, 1) != 0x80) {
    return false;
  }
  return true;
}


bool read(uint8_t reg_add, uint8_t* buff , uint8_t len)
{
    Wire.beginTransmission((uint8_t)MYAHRS_I2C_ADDRESS);  
        
    Wire.write(reg_add); 
    
    Wire.endTransmission(false); 

    Wire.requestFrom((uint8_t)MYAHRS_I2C_ADDRESS, len);

    uint8_t cnt = 0;
    while(Wire.available()) {
        buff[cnt++] = Wire.read();
    }

    return (cnt == len);
}

bool write(uint8_t reg_add, uint8_t* buff , uint8_t len)
{
    Wire.beginTransmission((uint8_t)MYAHRS_I2C_ADDRESS);
    
    Wire.write(reg_add);                          

    for(uint8_t cnt=0; cnt<len; cnt++) {
        Wire.write(buff[cnt]);                   
    }
    
    return Wire.endTransmission(true) == 0;   
}

int read_raw_data()
{
  uint8_t buf_raw_data[18];
  
  read(I2C_SLAVE_REG_I_ACC_X_LOW, buf_raw_data, 18);
  
  //Little endian
  int16_t acc_x = (buf_raw_data[1]<<8) | buf_raw_data[0];
  int16_t acc_y = (buf_raw_data[3]<<8) | buf_raw_data[2];
  int16_t acc_z = (buf_raw_data[5]<<8) | buf_raw_data[4];
  int16_t gyro_x = (buf_raw_data[7]<<8) | buf_raw_data[6];
  int16_t gyro_y = (buf_raw_data[9]<<8) | buf_raw_data[8];
  int16_t gyro_z = (buf_raw_data[11]<<8) | buf_raw_data[10];
  int16_t mag_x = (buf_raw_data[13]<<8) | buf_raw_data[12];
  int16_t mag_y = (buf_raw_data[15]<<8) | buf_raw_data[14];
  int16_t mag_z = (buf_raw_data[17]<<8) | buf_raw_data[16];
  
}

int read_compensated_data()
{
  uint8_t buf_comp_data[18];
  
  read(I2C_SLAVE_REG_C_ACC_X_LOW, buf_comp_data, 18);
  
  int16_t acc_c_x = (buf_comp_data[1]<<8) | buf_comp_data[0];
  int16_t acc_c_y = (buf_comp_data[3]<<8) | buf_comp_data[2];
  int16_t acc_c_z = (buf_comp_data[5]<<8) | buf_comp_data[4];
  int16_t gyro_c_x = (buf_comp_data[7]<<8) | buf_comp_data[6];
  int16_t gyro_c_y = (buf_comp_data[9]<<8) | buf_comp_data[8];
  int16_t gyro_c_z = (buf_comp_data[11]<<8) | buf_comp_data[10];
  int16_t mag_c_x = (buf_comp_data[13]<<8) | buf_comp_data[12];
  int16_t mag_c_y = (buf_comp_data[15]<<8) | buf_comp_data[14];
  int16_t mag_c_z = (buf_comp_data[17]<<8) | buf_comp_data[16];
  
  float comp_acc_x = (float)acc_c_x * 16.0 / 32767;
  float comp_acc_y = (float)acc_c_y * 16.0 / 32767;
  float comp_acc_z = (float)acc_c_z * 16.0 / 32767;
  float comp_gyro_x = (float)gyro_c_x * 2000 / 32767;
  float comp_gyro_y = (float)gyro_c_y * 2000 / 32767;
  float comp_gyro_z = (float)gyro_c_z * 2000 / 32767;
  float comp_mag_x = (float)mag_c_x * 0.3;
  float comp_mag_y = (float)mag_c_y * 0.3;
  float comp_mag_z = (float)mag_c_z * 0.3;
}



// EULER ANGLE(RPY)
int read_euler()
{
  uint8_t buf_euler[6];
  
  read(I2C_SLAVE_REG_ROLL_LOW, buf_euler, 6);
  
  int16_t euler_x = (buf_euler[1]<<8) | buf_euler[0];
  int16_t euler_y = (buf_euler[3]<<8) | buf_euler[2];
  int16_t euler_z = (buf_euler[5]<<8) | buf_euler[4];

  float roll = (float)euler_x * 180 / 32767;
  float pitch = (float)euler_y * 180 / 32767;
  float yaw = (float)euler_z * 180 / 32767;
  
  imu_data.x = roll;
  imu_data.y = pitch;
  imu_data.z = yaw;
  RPY.publish(&imu_data);
}

int read_quat()
{
  uint8_t buf_quat[8];
  
  read(I2C_SLAVE_REG_QUATERNIAN_X_LOW, buf_quat, 8);
  
  int16_t quat_x = (buf_quat[1]<<8) | buf_quat[0];
  int16_t quat_y = (buf_quat[3]<<8) | buf_quat[2];
  int16_t quat_z = (buf_quat[5]<<8) | buf_quat[4];
  int16_t quat_w = (buf_quat[7]<<8) | buf_quat[6];
  
  float quaternion_x = (float)quat_x / 32767;
  float quaternion_y = (float)quat_y / 32767;
  float quaternion_z = (float)quat_z / 32767;
  float quaternion_w = (float)quat_w / 32767;
  
}
//
////
 //READ REVISION ID
int read_rev_id()
{
  uint8_t id_1 = 0;
  uint8_t id_2 = 0;
  
  read(I2C_SLAVE_REG_REV_ID_MAJOR, &id_1, 1);
  read(I2C_SLAVE_REG_REV_ID_MAJOR, &id_2, 1);

}

int who_am_i()
{
  uint8_t whomi = 0;
  
  read(I2C_SLAVE_REG_WHO_AM_I, &whomi, 1);

}


////////////////// End IMU ///////////////////////////////




void loop(){

  read_sensor_values();
  
  if(gripper_mode_data == 0){
      mode = 0;
      current_mode.data = mode;
      check_mode.publish(&current_mode);
      //idle_state();
  }    
  else if(gripper_mode_data == 1){   
      mode = 1;
      current_mode.data = mode;
      check_mode.publish(&current_mode);
      //idle_state();
      //valve_pinch();
  }
  else if(gripper_mode_data == 2){
     mode = 3;
     current_mode.data = mode;
     check_mode.publish(&current_mode);
     close_grip();
  }
  
 else if(gripper_mode_data == 3){
     mode = 5;
     current_mode.data = mode;
     check_mode.publish(&current_mode);
     open_grip();
  } 
  grip.spinOnce();
}


void read_sensor_values(){
  
    read_euler();

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

 //   if ( (millis()-range_timer) > 20){
      pushed_msg1.data = reading1;
      pub_button1.publish(&pushed_msg1);
      pushed_msg2.data = reading2;
      pub_button2.publish(&pushed_msg2);
      pushed_msg3.data = reading3;
      pub_button3.publish(&pushed_msg3);
      pushed_msg4.data = reading4;
      pub_button4.publish(&pushed_msg4);
//       range_timer = millis();
//    }
    grip.spinOnce();      
}


void valve_pinch(){
  
  pinch_again_local = pinch_again;
  obs[0] = analogRead(A0);
  
  for (int i=max_pos; i>=min_pos_front; i -= 3){
  
      Servo_front.writeMicroseconds(i);
      delay(100);
     
      total = total - readings[readIndex];
      readings[readIndex] = analogRead(A0);
      total = total + readings[readIndex];
      readIndex = readIndex + 1;
      if (readIndex >= numReadings) {
      readIndex = 0;
      }
      average = total / numReadings;
  
      delay(100);
      obs[1] = average;
      if ((abs(obs[1]-obs[0])<1 && i<1800)||average>600){
        Servo_front.writeMicroseconds(i+100);
         break;        
      }     
      obs[0] = obs[1];
      read_sensor_values();
      grip.spinOnce();      
    }
    
   readIndex = 0;
   total = 0;                 
   average = 0;
   pinch_again_local = pinch_again_local + 1;
   
   mode = 2;
   current_mode.data = mode;
   check_mode.publish(&current_mode);
    
   while(gripper_mode_data==1){
     
        delay(50);
        read_sensor_values();
        update_mode(mode);
        grip.spinOnce();
        if (pinch_again==pinch_again_local){
            for (int i=1; i<=10; i++){
                delay(50);
                read_sensor_values();
                update_mode(mode);
                grip.spinOnce();
                Servo_front.writeMicroseconds(max_pos);
             }
            valve_pinch();
        }  
    }
   pinch_again_local = 0; 
}



void close_grip(){
   
   for (int j=max_pos; j>=min_pos_front; j -= 5){ 
    Servo_front.writeMicroseconds(j);
    delay(10);
    read_sensor_values();
    grip.spinOnce();
   }
    obs[0] = analogRead(A2);
    for (int i=max_pos; i>=min_pos_back; i -= 10){
  
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
  
      delay(100);
      obs[1] = average;
      if ((abs(obs[1]-obs[0])<1 && i<1800)||average>530){
        Servo_back.writeMicroseconds(i+100);
         break;        
      }     
      obs[0] = obs[1];
      read_sensor_values();
      update_mode(mode);
      grip.spinOnce();      
    }
        
    readIndex = 0;
    total = 0;                 
    average = 0;
    
    mode = 4;
    current_mode.data = mode;
    check_mode.publish(&current_mode);
    
    while(gripper_mode_data==2){
        delay(50);
        read_sensor_values();
        update_mode(mode);
        grip.spinOnce();
    }
    open_grip();
}


void open_grip(){
  
  for (int j=min_pos_back; j<=max_pos;j+=5){ 
    Servo_back.writeMicroseconds(j);
    delay(50);
    read_sensor_values();
    update_mode(mode);
    grip.spinOnce();
   }

  for (int i=min_pos_front; i<=max_pos; i+=5){
    delay(50);
    read_sensor_values();
    update_mode(mode);
    grip.spinOnce();
    Servo_front.writeMicroseconds(i);
  }
  
   mode = 6;
   current_mode.data = mode;
   check_mode.publish(&current_mode);
   
   while(gripper_mode_data==3){
     delay(50);
     read_sensor_values();
     update_mode(mode);
     grip.spinOnce();
   }   
}


void idle_state(){
  
  for (int j=min_pos_back; j<=max_pos;j+=5){ 
    Servo_back.writeMicroseconds(j);
    delay(50);
    read_sensor_values();
    update_mode(mode);
    grip.spinOnce();
   }

  for (int i=min_pos_front; i<=max_pos; i+=5){
    delay(50);
    read_sensor_values();
    update_mode(mode);
    grip.spinOnce();
    Servo_front.writeMicroseconds(i);
  }
  
   mode = 0;
   
   current_mode.data = mode;
   check_mode.publish(&current_mode);
   
   while(gripper_mode_data==3){
     delay(50);
     read_sensor_values();
     update_mode(mode);
     grip.spinOnce();
   }
}


void update_mode(int md){
  current_mode.data = md;
  check_mode.publish(&current_mode);
}
