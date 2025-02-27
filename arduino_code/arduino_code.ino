#include <PID_v1.h> 
#include <ros.h>
#include <ros/time.h> 
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "GyverMotor.h"

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
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

GMotor motorR(DRIVER2WIRE, 8, 9, HIGH);
GMotor motorL(DRIVER2WIRE, 4, 10, HIGH);

double Pk = 50, Ik = 5, Dk = 0.1;

double Pk1 = Pk;
double Ik1 = Ik;
double Dk1 = Dk;

double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);

double Pk2 = Pk;
double Ik2 = Ik;
double Dk2 = Dk;

double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

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
  // put your setup code here, to run once:
  motorR.setMode(AUTO);
  motorL.setMode(AUTO);
  
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
  Serial.begin(57600);
}

void loop()
{
    nh.spinOnce();

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

    motorR.setSpeed(Output1 > 0 ? -abs(Output1) : abs(Output1));
    motorL.setSpeed(Output2 > 0 ? -abs(Output2) : abs(Output2));

    //delay(10);
}

void doEncoderRPinA(){
  if (digitalRead(encoderRPinA) == digitalRead(encoderRPinB)) {
    encoderR_Pos--;  // В одну сторону
  } else {
    encoderR_Pos++;  // В другую сторону
  }                                  
}


void doEncoderLPinA(){
     if (digitalRead(encoderLPinA) == digitalRead(encoderLPinB)) {
    encoderL_Pos++;  // В одну сторону
  } else {
    encoderL_Pos--;  // В другую сторону
  }
   }
