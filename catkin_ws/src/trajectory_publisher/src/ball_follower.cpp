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
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

// Adjustable set height off from starting position
#define dz 0.2

volatile bool recievingPosition;
volatile bool extPosReady;
volatile bool crazyflieReady;
volatile bool swingingUp;
volatile bool firstSwingAdjust = false;

volatile double xS;
volatile double yS;
volatile double zS;
volatile double xD;
volatile double yD;
volatile double zD;

volatile double lbx = 0;
volatile double lby = 0;
volatile double lbz = 0;

volatile double vx = 0;
volatile double vy = 0;
volatile double vz = 0;
volatile double t = 0;

ros::Time lstamp;
ros::Time lastPredictTime;
ros::Duration dt;
double alpha = 0.5;
bool acceptBallSetpoint = false;
bool forwardPredict = true;

void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if( !recievingPosition ) {
        recievingPosition = true;
        xS = msg->point.x;
        yS = msg->point.y;
        zS = msg->point.z;
    }
}

void ballCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {

    if (acceptBallSetpoint) {

        if( forwardPredict ) {
            if( ros::Time::now() - lastPredictTime < ros::Duration(1) ) {
                // Updtate time increment
                dt = msg->header.stamp - lstamp;
                lstamp = msg->header.stamp;

                // Apply exponentially moving average filter to velocity approximations
                vx = alpha*( msg->point.x - lbx )/dt.toSec() + (1-alpha)*vx;
                vy = alpha*( msg->point.y - lby )/dt.toSec() + (1-alpha)*vy;
                vz = alpha*( msg->point.z - lbz )/dt.toSec() + (1-alpha)*vz;

                // Calculated closed form value of time at which ball will cross the
                // quadcopter's z plane
                t = 0.102*(vz + sqrt( vz*vz + 19.62*msg->point.z - 19.62*zD));

                if ( ~isnan(t) && std::isfinite(t) ) {
                    // Solve for the x,y position at the time of impact
                    double x = msg->point.x + vx*t;
                    double y = msg->point.y + vy*t;

                    if( x < 2 && x > -2 && y < 1.5 && y > -1 ) {
                        xD = x;
                        yD = y;
                    }
                }
                lastPredictTime = ros::Time::now();
            } else {
                lstamp = msg->header.stamp;
                lastPredictTime = ros::Time::now();
            }
        } else {
            xD = msg->point.x;
            yD = msg->point.y;
        }

        // Update last point for next iteration
        lbx = msg->point.x;
        lby = msg->point.y;
        lbz = msg->point.z;

    } else {
        lstamp = msg->header.stamp;
        lbx = msg->point.x;
        lby = msg->point.y;
        lbz = msg->point.z;
    }
}

void readyCallback(const std_msgs::UInt8::ConstPtr& msg) {
    crazyflieReady = true;
}

void extPosReadyCallback(const std_msgs::UInt8::ConstPtr& msg) {
    extPosReady = true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ball_follower");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    ros::Subscriber quad_sub = n.subscribe<geometry_msgs::PointStamped>("/crazyflie/external_position",1,positionCallback);
    ros::Subscriber ball_sub = n.subscribe<geometry_msgs::PointStamped>("/ball/position",1,ballCallback);
    ros::Publisher  waypoint_pub = n.advertise<geometry_msgs::PointStamped>("/crazyflie/cmd_waypoint",1);
    ros::Publisher  target_pub = n.advertise<geometry_msgs::PointStamped>("/target_pos",1);
    ros::Subscriber ready_sub = n.subscribe<std_msgs::UInt8>("/crazyflie/ready",1,readyCallback);
    ros::Subscriber ext_ready = n.subscribe<std_msgs::UInt8>("/external_position/ready",1,extPosReadyCallback);
    geometry_msgs::PointStamped waypoint_msg;
    ros::Rate slow_loop(5);
    ros::Rate fast_loop(200);

    n_private.param<bool>("forwardPredict", forwardPredict, false);

    lstamp = ros::Time::now();
    lastPredictTime = ros::Time::now();
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

    // Define transform and its broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    ROS_INFO("Hovering at X(%f) Y(%f) Z(%f)...", xS, yS, zS+dz);
    // Set the copter to hover
    for( int i = 0; i < 50; i ++ ) {
        waypoint_msg.point.x = xS;
        waypoint_msg.point.y = yS;
        // Slowly ramp up the z setpoint to the desired height
        waypoint_msg.point.z = zS + dz*(1.0 - (49.0-(float)i)/49);
        // ROS_INFO("Commanding Z(%f)",zS + 0.2*(1.0 - (49.0-(float)i)/49));
        waypoint_pub.publish(waypoint_msg);
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        slow_loop.sleep();
    }

    lstamp = ros::Time::now();
    // Allow the ball setpoint tracker to run
    acceptBallSetpoint = true;
    // Set inital desired position (x and y updated async in ballCallback)
    xD = xS;
    yD = yS;
    zD = zS + dz;
    geometry_msgs::PointStamped msg;
    // Set the copter to swingup mode (changed in ballCallback)
    swingingUp = true;

    ros::Duration ct;
    ros::Time startTime = ros::Time::now();

    // Loop until ros quits
    while(ros::ok()) {

        // Track trajectory from forward balistic model
        waypoint_msg.header.stamp = ros::Time::now();
        waypoint_msg.point.x = xD;
        waypoint_msg.point.y = yD;
        // Dip in z to aid in catching
        waypoint_msg.point.z = zD;

        waypoint_pub.publish(waypoint_msg);

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "target";
        // Set relative position to zero
        transformStamped.transform.translation.x = xD;
        transformStamped.transform.translation.y = yD;
        transformStamped.transform.translation.z = zD;
        transformStamped.transform.rotation.w = 1;

        // Send off transform
        br.sendTransform(transformStamped);

        msg.header.frame_id ="target";
        msg.header.stamp = ros::Time::now();
        msg.point.x = xD;
        msg.point.y = yD;
        msg.point.z = zD;
        target_pub.publish(msg);
        ros::spinOnce();
        fast_loop.sleep();
    }

}
