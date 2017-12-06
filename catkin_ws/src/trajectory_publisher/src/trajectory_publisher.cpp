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
#include <std_msgs/UInt8.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

volatile bool recievingPosition;
volatile bool extPosReady;
volatile bool crazyflieReady;
volatile double xS;
volatile double yS;
volatile double zS;
volatile double xD;
volatile double yD;
volatile double zD;

void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if( !recievingPosition ) {
        recievingPosition = true;
        xS = msg->point.x;
        yS = msg->point.y;
        zS = msg->point.z;
    }
}

void ballCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    xD = msg->point.x;
    yD = msg->point.y;
    //zD = msg->point.z;
}

void readyCallback(const std_msgs::UInt8::ConstPtr& msg) {
    crazyflieReady = true;
}

void extPosReadyCallback(const std_msgs::UInt8::ConstPtr& msg) {
    extPosReady = true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    ros::Subscriber quad_sub = n.subscribe<geometry_msgs::PointStamped>("/crazyflie/external_position",1,positionCallback);
    ros::Subscriber ball_sub = n.subscribe<geometry_msgs::PointStamped>("/ball/position",1,ballCallback);
    ros::Publisher  waypoint_pub = n.advertise<geometry_msgs::PointStamped>("/crazyflie/cmd_waypoint",1);
    ros::Subscriber ready_sub = n.subscribe<std_msgs::UInt8>("/crazyflie/ready",1,readyCallback);
    ros::Subscriber ext_ready = n.subscribe<std_msgs::UInt8>("/external_position/ready",1,extPosReadyCallback);
    geometry_msgs::PointStamped waypoint_msg;
    ros::Rate slow_loop(10);
    ros::Rate fast_loop(200);

    crazyflieReady = false;

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

    ROS_INFO("Hovering at X(%f) Y(%f) Z(%f)...", xS, yS, zS);
    // Set the copter to hover
    for( int i = 0; i < 50; i ++ ) {
        waypoint_msg.point.x = xS;
        waypoint_msg.point.y = yS;
        waypoint_msg.point.z = zS + 0.2*(1.0 - (49.0-(float)i)/49);
        // ROS_INFO("Commanding Z(%f)",zS + 0.2*(1.0 - (49.0-(float)i)/49));
        waypoint_pub.publish(waypoint_msg);
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        slow_loop.sleep();
    }

    xD = xS;
    yD = yS;
    zD = zS + 0.2;

    // Loop until ros quits
    while(ros::ok()) {
        // ROS_INFO("Hovering X(%f) Y(%f) Z(%f)...", xD, yD, zD);
        waypoint_msg.header.stamp = ros::Time::now();
        waypoint_msg.point.x = xD;
        waypoint_msg.point.y = yD;
        waypoint_msg.point.z = zD;
        waypoint_pub.publish(waypoint_msg);

        ros::spinOnce();
        fast_loop.sleep();
    }

}
