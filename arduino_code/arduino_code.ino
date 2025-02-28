#include <PID_v1.h> 
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <GyverMotor.h>

GMotor motorR(DRIVER2WIRE, 8, 9, HIGH);
GMotor motorL(DRIVER2WIRE, 4, 10, HIGH);

ros::NodeHandle nh;

char base_link[]="/base_link";
char odom[]="/odom";

unsigned long currentMillis;
unsigned long previousMillis;

//wheel encoder
#define encoderRPinA 2
#define encoderRPinB 5
#define encoderLPinA 3
#define encoderLPinB 6

volatile long encoderR_Pos = 0;
volatile long encoderL_Pos = 0;


float Pk1 = 50.0, Ik1 = 5, Dk1 = 0.05;
float Pk2 = 50.0, Ik2 = 5, Dk2 = 0.05;

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

// ROS callback
void cmdVelCallback(const geometry_msgs::Twist &cmd)
{
    demandx = cmd.linear.x;
    demandz = cmd.angular.z;
    
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);
//geometry_msgs::Vector3Stamped speed_msg;
//ros::Publisher speed_pub("speed", &speed_msg);

geometry_msgs::Vector3Stamped motor_output_msg;
ros::Publisher motor_output_pub("motor_output", &motor_output_msg);
void setup() {
  motorR.setMode(AUTO);
  motorL.setMode(AUTO);
  
  pinMode(encoderRPinA, INPUT_PULLUP);
  pinMode(encoderRPinB, INPUT_PULLUP);
  pinMode(encoderLPinA, INPUT_PULLUP);
  pinMode(encoderLPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), doEncoderRPinA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderLPinA, CHANGE);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-250, 250);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-250, 250);
  PID2.SetSampleTime(10);
  nh.initNode();
  nh.advertise(motor_output_pub);
  nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
    currentMillis = millis();
    if(currentMillis - previousMillis >= 10){
      previousMillis = currentMillis;
      encoderRDiff = encoderR_Pos - encoderRPrev;
      encoderLDiff = encoderL_Pos - encoderLPrev;

      encoderRPrev = encoderR_Pos;
      encoderLPrev = encoderL_Pos;

      demand2 = demandx - (demandz * 0.131);
      demand1 = demandx + (demandz * 0.131);

      if(demand1>0 && demand2>0){
        Pk1 = 50.0, Ik1 = 5, Dk1 = 0.05;
        Pk2 = 50.0, Ik2 = 5, Dk2 = 0.05;
      }
      else if(demand1<0 && demand2<0){
        Pk1 = 50.0, Ik1 = 5, Dk1 = 0.05;
        Pk2 = 50.0, Ik2 = 5, Dk2 = 0.05;
      }
        else if(demand1<0 && demand2>0){
        Pk1 = 250.0, Ik1 = 50, Dk1 = 0.05;
        Pk2 = 50.0, Ik2 = 5, Dk2 = 0.05;
      }
        else if(demand1>0 && demand2<0){
        Pk1 = 50.0, Ik1 = 5, Dk1 = 0.05;
        Pk2 = 250.0, Ik2 = 50, Dk2 = 0.05;
      }
      
      Setpoint1 = demand1 * 10;
      Input1 = encoderRDiff;
      PID1.Compute();

      Setpoint2 = demand2 * 10;
      Input2 = encoderLDiff;
      PID2.Compute();

      motorR.setSpeed(-Output1);
      motorL.setSpeed(-Output2);

      motor_output_msg.header.stamp = nh.now();
      motor_output_msg.vector.x = Output1; // Выход PID для правого мотора
      motor_output_msg.vector.y = Output2; // Выход PID для левого мотора
      motor_output_pub.publish(&motor_output_msg);
    }

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
  //Serial.println(encoderL_Pos);
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
