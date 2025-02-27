#include <PID_v1.h> 
#include <ros.h>
#include <ros/time.h> 
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

tf::TransformBroadcaster broadcaster;

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

double Pk = 50, Ik = 5, Dk = 0.1;

double Setpoint1, Input1, Output1;
PID PID1(&Input1, &Output1, &Setpoint1, Pk, Ik, Dk, DIRECT);

double Setpoint2, Input2, Output2;
PID PID2(&Input2, &Output2, &Setpoint2, Pk, Ik, Dk, DIRECT);

float demand1;
float demand2;


float encoderRDiff;
float encoderLDiff;

float encoderRError;
float encoderLError;

float encoderRPrev;
float encoderLPrev;

// ROS callback
void cmdVelCallback(const geometry_msgs::Twist &cmd)
{
    float demandx = cmd.linear.x;
    float demandz = cmd.angular.z;
    
    demand1 = demandx - (demandz * 0.131);
    demand2 = demandx + (demandz * 0.131);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCallback);

void setup() {
  geometry_msgs::TransformStamped t;
  //motorR.setMode(AUTO);
  //motorL.setMode(AUTO);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(10, OUTPUT);
  
  pinMode(encoderRPinA, INPUT_PULLUP);
  pinMode(encoderRPinB, INPUT_PULLUP);
  pinMode(encoderLPinA, INPUT_PULLUP);
  pinMode(encoderLPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), doEncoderRPinA, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderLPinA, HIGH);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-250, 250);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-250, 250);
  PID2.SetSampleTime(10);
  nh.initNode();
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

      Setpoint1 = demand1 * 5;
      Input1 = encoderRDiff;
      PID1.Compute();

      Setpoint2 = demand2 * 5;
      Input2 = encoderLDiff;
      PID2.Compute();
      
      setMotorSpeed(1, Output1);  
      setMotorSpeed(2, Output2);
    }

}

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
}


void doEncoderRPinA(){
  if (digitalRead(encoderRPinA) == digitalRead(encoderRPinB)) {
    encoderR_Pos--;  
  } else {
    encoderR_Pos++;  
  }                                  
}


void doEncoderLPinA(){
     if (digitalRead(encoderLPinA) == digitalRead(encoderLPinB)) {
    encoderL_Pos++;  
  } else {
    encoderL_Pos--;  
  }
   }
