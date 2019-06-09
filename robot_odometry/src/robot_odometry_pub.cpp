#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double temp_th =0.0;

void write_callback_1(const geometry_msgs::Vector3::ConstPtr &msg)
{
	vx = (msg->y + msg->x) / 2;
  vth = msg->z;
  //vth = (msg->y - msg->x) / Diff;
}

void write_callback_2(const sensor_msgs::Imu::ConstPtr &msg)
{
  static double q0,q1,q2,q3;
  q0 = (msg->orientation).w;
  q1 = (msg->orientation).x;
  q2 = (msg->orientation).y;
  q3 = (msg->orientation).z;

  static double yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
  static double yaw_last = yaw;

  yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
  temp_th = yaw -yaw_last;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_odometry");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 500);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber vel_sub = n.subscribe("/vel_pub", 500, write_callback_1);

  ros::Subscriber imu_sub = n.subscribe("/imu/data", 500, write_callback_2);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  nav_msgs::Odometry odom;

  ros::Rate loop_rate(10);
  while(n.ok()){
	
    vx = 0.0;
    vy = 0.0;
    vth = 0.0;

    ros::spinOnce();               // check for incoming messages
    ros::spinOnce(); 
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_s = vx * dt;
	  double delta_th = vth * dt;
    //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

    //double delta_x = vx * cos(delta_th) * dt;
    //double delta_y = vx * sin(delta_th) * dt;

    x += delta_s * cos(th + delta_th / 2);
    y += delta_s * sin(th + delta_th / 2);
    //th += delta_th;
    th = temp_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    static geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    //nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    loop_rate.sleep();
  }
}