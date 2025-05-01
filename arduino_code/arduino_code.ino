#include <Wire.h>
#include <MPU6050.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <PID_v1.h>
#include <GyverMotor.h>
#include <MadgwickAHRS.h>

Madgwick filter;
MPU6050 mpu;
unsigned long lastUpdate = 0;
float gX_offset = 0, gY_offset = 0, gZ_offset = 0;


GMotor motorR(DRIVER2WIRE, 6, 7, HIGH);
GMotor motorL(DRIVER2WIRE, 10, 5, HIGH);

ros::NodeHandle nh;
std_msgs::Int16MultiArray qecounts;
std_msgs::Float32MultiArray imuRead;


unsigned long currentMillis;
unsigned long previousMillis;
unsigned long previousMillis2;

//wheel encoder
#define encoderRPinA 3
#define encoderRPinB 2
#define encoderLPinA 19
#define encoderLPinB 18

volatile long encoderR_Pos = 0;
volatile long encoderL_Pos = 0;
volatile long encoderR_Pos_accum = 0;
volatile long encoderL_Pos_accum = 0;

float Pk1 = 35, Ik1 = 40, Dk1 = 0.02;
float Pk2 = 35, Ik2 = 40, Dk2 = 0.02;

double Setpoint1, Input1, Output1;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);

double Setpoint2, Input2, Output2;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

float demand1;
float demand2;


float encoderRDiff;
float encoderLDiff;

float encoderRError;
float encoderLError;

float encoderRPrev;
float encoderLPrev;

float demandx;
float demandz;

double rpm1, rpm2;
// ROS callback
void cmdVelCallback(const geometry_msgs::Twist &cmd)
{
    demandx = cmd.linear.x;
    demandz = cmd.angular.z;
    
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);
ros::Publisher quadenc("quadenc", &qecounts);
ros::Publisher imupub("imupub", &imuRead);

void setup() {
  Wire.begin();
  mpu.initialize();
  filter.begin(20);
  calibrateGyro();
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;  // Пины 5 (Timer 3) - 31.25 кГц
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;  // Пины 6, 7 (Timer 4) - 31.25 кГц
  
  motorR.setMode(AUTO);
  motorL.setMode(AUTO);
  
  pinMode(encoderRPinA, INPUT_PULLUP);
  pinMode(encoderRPinB, INPUT_PULLUP);
  pinMode(encoderLPinA, INPUT_PULLUP);
  pinMode(encoderLPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(3), doEncoderRPinA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderRPinB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), doEncoderLPinA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), doEncoderLPinB, CHANGE);
  
  static int enc_data[2];
  qecounts.data = enc_data;
  qecounts.data_length = 2;

  static float imu_data[10];
  imuRead.data = imu_data;
  imuRead.data_length = 10;

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-250, 250);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-250, 250);
  PID2.SetSampleTime(10);
  nh.initNode();
  nh.advertise(quadenc);
  nh.advertise(imupub);
  nh.subscribe(sub);
  //Serial.begin(115200);
}

void loop()
{
    nh.spinOnce();
    currentMillis = millis();
    if(currentMillis - previousMillis >= 10){
      previousMillis = currentMillis;

      encoderRDiff = encoderR_Pos - encoderRPrev;
      encoderLDiff = encoderL_Pos - encoderLPrev;
      encoderR_Pos_accum += encoderRDiff;
      encoderL_Pos_accum += encoderLDiff;
      
      if(currentMillis - previousMillis2 >= 50){
          qecounts.data[0] = encoderR_Pos_accum;
          qecounts.data[1] = encoderL_Pos_accum;
          encoderR_Pos_accum = 0;
          encoderL_Pos_accum = 0;
          previousMillis2 = currentMillis;
          int16_t ax, ay, az, gx, gy, gz;
          mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

          float aX = ax / 16384.0, aY = ay / 16384.0, aZ = az / 16384.0;
          //float gX = gx / 131.0, gY = gy / 131.0, gZ = gz / 131.0;

          float gX = gx / 131.0 - gX_offset;
          float gY = gy / 131.0 - gY_offset;
          float gZ = gz / 131.0 - gZ_offset;
          
          if (abs(gX) < 0.05) gX = 0;
          if (abs(gY) < 0.05) gY = 0;
          if (abs(gZ) < 0.05) gZ = 0;
          filter.updateIMU(gX, gY, gZ, aX, aY, aZ);

          float qx, qy, qz, qw;
          filter.getQuaternion(&qw, &qx, &qy, &qz);

          imuRead.data[0] = qx;
          imuRead.data[1] = qy;
          imuRead.data[2] = qz;
          imuRead.data[3] = qw;

          imuRead.data[4] = gX;
          imuRead.data[5] = gY;
          imuRead.data[6] = gZ;

          imuRead.data[7] = aX;
          imuRead.data[8] = aY;
          imuRead.data[9] = aZ;

          imupub.publish(&imuRead);
          quadenc.publish(&qecounts);
      }
      
      encoderRPrev = encoderR_Pos;
      encoderLPrev = encoderL_Pos;

      demand2 = demandx - (demandz * 0.131);
      demand1 = demandx + (demandz * 0.131);
      
      Setpoint1 = demand1 * 10;
      Input1 = encoderRDiff;
      PID1.Compute();

      Setpoint2 = demand2 * 10;
      Input2 = encoderLDiff;
      PID2.Compute();

      motorR.setSpeed(Output1);
      motorL.setSpeed(Output2);

      

      //Serial.print(encoderL_Pos);
      //Serial.print(", ");
      //Serial.println(encoderR_Pos);
      //publishSpeed(10);
      /*
      motor_output_msg.header.stamp = nh.now();
      motor_output_msg.vector.x = Output1; // Выход PID для правого мотора
      motor_output_msg.vector.y = Output2; // Выход PID для левого мотора
      motor_output_pub.publish(&motor_output_msg);  
    */ 
      
    }

}
void calibrateGyro() {
  long gx = 0, gy = 0, gz = 0;
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, temp;
    int16_t gx_raw, gy_raw, gz_raw;
    mpu.getMotion6(&ax, &ay, &az, &gx_raw, &gy_raw, &gz_raw);
    gx += gx_raw;
    gy += gy_raw;
    gz += gz_raw;
    delay(5);
  }
  gX_offset = gx / 100.0 / 131.0;
  gY_offset = gy / 100.0 / 131.0;
  gZ_offset = gz / 100.0 / 131.0;
}

/*
void setMotorSpeed(int motor, int speed) {
  if (motor == 1) {  // Для правого мотора
    if (speed > 0) {
      analogWrite(8, speed);    // Подаем сигнал на один пин
      analogWrite(9, 0);        // Отключаем противоположный пин
    } else if (speed < 0) {
      analogWrite(8, 0);
      analogWrite(9, -speed);   // Подаем сигнал на противоположный пин
    } else {
      analogWrite(8, 0);
      analogWrite(9, 0);         // Останавливаем мотор
    }
  } else if (motor == 2) {  // Для левого мотора
    if (speed > 0) {
      analogWrite(4, speed);
      analogWrite(10, 0);
    } else if (speed < 0) {
      analogWrite(4, 0);
      analogWrite(10, -speed);
    } else {
      analogWrite(4, 0);
      analogWrite(10, 0);
    }
  }
}*/
/*
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();
  speed_msg.header.stamp.sec -= 0;
  speed_msg.header.stamp.nsec -= 10000000; // Коррекция на 10 мс

  speed_msg.vector.x = speed_act_left;  
  speed_msg.vector.y = speed_act_right;
  speed_msg.vector.z = time / 1000.0; 

  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
*/
void doEncoderRPinA(){
  if (digitalRead(encoderRPinA) == HIGH){
    if (digitalRead(encoderRPinB) == LOW){
      encoderR_Pos ++;
    }
    else{
      encoderR_Pos --;
    }
  }
  else {
    if (digitalRead(encoderRPinB) == HIGH){
      encoderR_Pos ++;
    }
    else{
      encoderR_Pos --;
    }
  }
}

void doEncoderRPinB(){
  if (digitalRead(encoderRPinB) == HIGH){
    if (digitalRead(encoderRPinA) == HIGH){
      encoderR_Pos ++;
    }
    else{
      encoderR_Pos --;
    }
  }
  else {
    if (digitalRead(encoderRPinA) == LOW){
      encoderR_Pos ++;
    }
    else{
      encoderR_Pos --;
    }
  }
}

void doEncoderLPinA(){
  if (digitalRead(encoderLPinB) == HIGH){
    if (digitalRead(encoderLPinA) == HIGH){
      encoderL_Pos ++;
    }
    else{
      encoderL_Pos --;
    }
  }
  else {
    if (digitalRead(encoderLPinA) == LOW){
      encoderL_Pos ++;
    }
    else{
      encoderL_Pos --;
    }
  }
  //Serial.println(encoderL_Pos);
}
void doEncoderLPinB(){
  if (digitalRead(encoderLPinB) == HIGH){
    if (digitalRead(encoderLPinA) == HIGH){
      encoderL_Pos --;
    }
    else{
      encoderL_Pos ++;
    }
  }
  else {
    if (digitalRead(encoderLPinA) == LOW){
      encoderL_Pos --;
    }
    else{
      encoderL_Pos ++;
    }
  }
}
