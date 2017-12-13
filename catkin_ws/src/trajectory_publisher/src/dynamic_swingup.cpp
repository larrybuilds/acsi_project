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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "crazyflie_driver/HoverStamped.h"
#include <std_msgs/UInt8.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include "std_srvs/Empty.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

// Adjustable set height off from starting position
#define dz 0.5
#define PEN_LEN 0.35
#define FREQ sqrt(9.81/PEN_LEN)
#define VEL_THRESH 3.0

volatile bool recievingPosition;
volatile bool extPosReady;
volatile bool crazyflieReady;
volatile bool swingingUp;

volatile double xS;
volatile double yS;
volatile double zS;
volatile double xD;
volatile double yD;
volatile double zD;
volatile double qx;
volatile double qy;
volatile double qz;

volatile double frequency;
volatile double amp;

ros::Duration ct;
ros::Time startTime;

void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if( !recievingPosition ) {
        recievingPosition = true;
        xS = msg->point.x;
        yS = msg->point.y;
        zS = msg->point.z;
    } else {
        qx = msg->point.x;
        qy = msg->point.y;
        qz = msg->point.z;
    }
}

void readyCallback(const std_msgs::UInt8::ConstPtr& msg) {
    crazyflieReady = true;
}

void extPosReadyCallback(const std_msgs::UInt8::ConstPtr& msg) {
    extPosReady = true;
}

void freqCallback(const std_msgs::Float32::ConstPtr& msg) {
    frequency = msg->data;
}

void ampCallback(const std_msgs::Float32::ConstPtr& msg) {
    amp = msg->data;
}

bool haltServiceCallback(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res)
{
  swingingUp = false;
  return true;
}

bool resumeServiceCallback(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res)
{
  swingingUp = true;
  startTime = ros::Time::now();
  return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    crazyflieReady = false;
    recievingPosition = false;
    extPosReady = false;

    ros::Subscriber quad_sub  = n.subscribe<geometry_msgs::PointStamped>("/crazyflie/external_position",1,positionCallback);
    ros::Subscriber ready_sub = n.subscribe<std_msgs::UInt8>("/crazyflie/ready",1,readyCallback);
    ros::Subscriber ext_ready = n.subscribe<std_msgs::UInt8>("/external_position/ready",1,extPosReadyCallback);
    ros::Subscriber freq_sub  = n.subscribe<std_msgs::Float32>("/freq",1,freqCallback);
    ros::Subscriber amp_sub   = n.subscribe<std_msgs::Float32>("/amp",1,ampCallback);

    ros::Publisher  waypoint_pub = n.advertise<geometry_msgs::PointStamped>("/crazyflie/cmd_waypoint",1);
    ros::Publisher  target_pub   = n.advertise<geometry_msgs::PointStamped>("/target_pos",1);
    ros::Publisher  xyKp_pub     = n.advertise<std_msgs::Float32>("/crazyflie/xyKp",1);
    ros::Publisher  hov_pub      = n.advertise<crazyflie_driver::HoverStamped>("/crazyflie/cmd_hover",1);

    ros::ServiceServer srvHalt   = n.advertiseService("/crazyflie/halt", &haltServiceCallback);
    ros::ServiceServer srvResume = n.advertiseService("/crazyflie/resume", &resumeServiceCallback);

    std_msgs::Float32 xyKp_msg;
    geometry_msgs::PointStamped waypoint_msg;
    geometry_msgs::PointStamped home_msg;
    crazyflie_driver::HoverStamped hov_msg;

    ros::Rate slow_loop(5);
    ros::Rate fast_loop(200);

    // Wait for the first position to come in
    ROS_INFO("Waiting for initial position...");
    while( !recievingPosition ) {
        ros::spinOnce();
        slow_loop.sleep();
    }

    ROS_INFO("Waiting for crazyflie...");
    while( !crazyflieReady ) {
        ros::spinOnce();
        slow_loop.sleep();
    }

    ROS_INFO("Waiting for external position...");
    while( !extPosReady ) {
        ros::spinOnce();
        slow_loop.sleep();
    }

    ROS_INFO("Taking off...");
    for( int i = 0; i < 30; i ++ ) {
        hov_msg.vx = 0;
        hov_msg.vy = 0;
        hov_msg.yawrate = 0;
        hov_msg.zDistance = zS + dz*(1.0 - (30-(float)i)/30);
        hov_pub.publish(hov_msg);
        ros::spinOnce();
        ros::spinOnce();
        slow_loop.sleep();
    }

    ROS_INFO("Sending static setpoint...");
    xS = qx;
    yS = qy;
    home_msg.point.x = xS;
    home_msg.point.y = yS;
    home_msg.point.z = zS + dz;
    // Hover at desired point for 3 seconds to allow settling of ball movement
    for( int i = 0; i < 30; i ++ ) {
        waypoint_pub.publish(home_msg);
        ros::spinOnce();
        slow_loop.sleep();
    }

    // Update the xy position P gain for faster response
    xyKp_msg.data = 4.0;
    xyKp_pub.publish(xyKp_msg);

    // Set inital desired position (x and y updated async in ballCallback)
    xD = xS;
    yD = yS;
    zD = zS + dz;
    geometry_msgs::PointStamped msg;
    msg.header.frame_id ="target";

    // Initialize freq and amp of sine wave
    frequency = 4.5;
    amp = 0.5;
    ROS_INFO("Starting Freq(%f) Amp(%f)", frequency, amp);
    swingingUp = false;
    startTime = ros::Time::now();

    // Loop until ros quits
    while(ros::ok()) {

        // Update current time
        ct = ros::Time::now() - startTime;

        // Run sinusoidal swingup at pendulum natrual freqency
        if( swingingUp ) {
            ct = ros::Time::now() - startTime;
            // Fill in the desired setpont with the original x and z, sinusoidal reference in y
            waypoint_msg.header.stamp = ros::Time::now();
            waypoint_msg.point.x = xS;
            //waypoint_msg.point.y = yS + sin(8.087026648*ct.toSec());      // 15cm
            //waypoint_msg.point.y = yS + 0.5*sin(5.718391382*ct.toSec());  // 30cm
            waypoint_msg.point.y = yS + (amp + exp(-0.01*ct.toSec()))*sin(frequency*ct.toSec());    // 28cm
            waypoint_msg.point.z = zD;
            waypoint_pub.publish(waypoint_msg);
        } else {
            // Send waypoint to quad
            waypoint_pub.publish(home_msg);
        }

        ros::spinOnce();
        fast_loop.sleep();
    }

}
