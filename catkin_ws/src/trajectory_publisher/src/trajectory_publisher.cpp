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
#include <gemoetry_msgs/PoseStamped.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

int main(int argc, char **argv){
  ros::init(argc, argv, "trajectory_publisher");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  // Variable definitions
  float amp, freq, x_off, y_off, z_off;

  // Pull parameters from server
  n_private.param<float>("amplitude", amp,   1);
  n_private.param<float>("freqency",  freq,  1);
  n_private.param<float>("x_offset",  x_off, 1);
  n_private.param<float>("y_offset",  y_off, 1);
  n_private.param<float>("z_offset",  z_off, 1);

  // Define waypoint publisher
  ros::Publisher pose_pub = n.advertize<gemoetry_msgs::PoseStamped>("/goal", 100);
  // Define current waypoint pose message
  gemoetry_msgs::PoseStamped pose_msg;

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
  startTime = ros::Time::now();
  ros::Rate looprate(2);

  // Set the copter to hover for 10s on start
  while( (ros::Time::now() - startTime) <= ros::Duration(10) ) {
    pose_pub.publish(pose_msg);
    ros::spinOnce();
    looprate.sleep();
  }

  // Loop until ros quits
  while(ros::ok()) {
    // Grab current time
    t = ros::Time::now();
    // Update trajectory
    pose_msg.pose.position.x = x_off + amp*sin(freq*(t.sec + 1e9*t.nsec));
    pose_msg.pose.position.y = y_off;
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
