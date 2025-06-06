#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>

 double qe1 = 0;
 double qe2 = 0;
 double dt = 0;
 double dx = 0;
 double dy = 0;
 double dth = 0;
 double dist = 0;
 double x = 0.0;
 double y = 0.0;
 double th = 0.0;
 double vx = 0.0;
 double vy = 0.0;
 double vth = 0.0;
 double dxp, dyp, vartheta, rcurv;
 ros::Time current_time, last_time;
 
 ros::Publisher odom_pub;
 
void quadencCallback(const std_msgs::Int16MultiArray& msg){
  tf::TransformBroadcaster odom_broadcaster;
 current_time = ros::Time::now();
 qe1 = msg.data[0]*0.00098947800113;//0.00007302800423;//0.00007356235548;//0.00007391858965;//0.00007427482381;//0.00007436388235;//fr //convert qecounts into m 
 qe2 = msg.data[1]*0.00098947800113;//0.00007302800423;//0.00007356235548;//0.00007391858965;//0.00007427482381;//0.00007432825894; //fl

dist = (qe1+qe2)/2;
dth = (qe1-qe2)/2;
dth = dth/.13; //convert to radians of rotation .14605m is 11.5in/2 (half of wheel base) [.147701 is a calibrated value]
dx = dist*cos(th);
dy = dist*sin(th);
x += dx;
y += dy;
th += dth;

 //we need a quaternion to describe rotation in 3d
 geometry_msgs::Quaternion odom_quat =tf::createQuaternionMsgFromYaw(th);
 
 dt =(current_time-last_time).toSec(); //calc velocities
 vx = dist/dt; //v is in base_link frame
 vy = 0;
 vth = dth/dt;
 
 nav_msgs::Odometry odom; //create nav_msgs::odometry 
 odom.header.stamp = current_time;
 odom.header.frame_id = "odom";
 
 odom.pose.pose.position.x = x; //set positions 
 odom.pose.pose.position.y = y;
 odom.pose.pose.position.z = 0.0;
 odom.pose.pose.orientation = odom_quat;
 
 odom.child_frame_id = "base_link"; // set child frame and set velocity in twist message
 odom.twist.twist.linear.x =vx;
 odom.twist.twist.linear.y =vy;
 odom.twist.twist.angular.z =vth;
 
 odom_pub.publish(odom);  //publish odom message
 

 last_time = current_time;
}

int main(int argc, char** argv){
 ros::init(argc, argv, "odometry_publisher");
 tf::TransformBroadcaster odom_broadcaster;
 ros::NodeHandle nh; 
 odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
 ros::Subscriber quadenc_sub = nh.subscribe("quadenc",50,quadencCallback);


 
 current_time = ros::Time::now();
 last_time = ros::Time::now();

 
 ros::spin();
 return 0;
 }

