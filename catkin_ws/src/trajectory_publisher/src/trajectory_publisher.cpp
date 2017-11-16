/******************************************************************************
    * trajectory_publisher.cpp -- Publishes trajectory waypoints to crazyflie *
    *                             ROS controller                              *
    * Carnegie Mellon Univesity                                               *
    *                                                                         *
    * Author:   Papincak, Lawrence                                            *
    *                                                                         *
    * Purpose:  Sends waypoints to crazyflie quadrotor that is operating in a *
    *           Optitrack MOCCAP system                                       *
    * Usage:                                                                  *
    **************************************************************************/

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "trajectory_publisher");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  // Variable definitions
  float amp, freq, x_off, y_off, z_off;
  uint32_t sec, nsec;

  // Pull parameters from server
  n_private.param<float>("amplitude",  amp,   1);
  n_private.param<float>("frequency",  freq,  1);
  n_private.param<float>("x_offset",   x_off, 1);
  n_private.param<float>("y_offset",   y_off, 1);
  n_private.param<float>("z_offset",   z_off, 1);

  // Define waypoint publisher
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/crazyflie/goal", 100);
  // Define current waypoint pose message
  geometry_msgs::PoseStamped pose_msg;

  // Initialize message for takeoff sequence
  pose_msg.pose.position.x = x_off;
  pose_msg.pose.position.y = y_off;
  pose_msg.pose.position.z = z_off;
  pose_msg.pose.orientation.x = 0;
  pose_msg.pose.orientation.y = 0;
  pose_msg.pose.orientation.z = 0;
  pose_msg.pose.orientation.w = 1;

  ros::Time startTime;
  ros::Time t;
  ros::Duration dt;

  startTime = ros::Time::now();
  ros::Rate looprate(5);

  // Set the copter to hover for 10s on start
  while( (ros::Time::now() - startTime) <= ros::Duration(20) ) {
    pose_msg.header.stamp = ros::Time::now();
    pose_pub.publish(pose_msg);
    ros::spinOnce();
    looprate.sleep();
  }

  startTime = ros::Time::now();
  // Loop until ros quits
  while(ros::ok()) {
    // Grab current time
    dt = ros::Time::now() - startTime;

    // Update trajectory
    pose_msg.header.stamp = t;
    pose_msg.pose.position.x = x_off;
    pose_msg.pose.position.y = y_off  + amp*sin(freq*dt.toSec());
    pose_msg.pose.position.z = z_off;
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = 0;
    pose_msg.pose.orientation.w = 1;

    // Publish waypoint
    pose_pub.publish(pose_msg);

    ros::spinOnce();
    looprate.sleep();
  }

}
