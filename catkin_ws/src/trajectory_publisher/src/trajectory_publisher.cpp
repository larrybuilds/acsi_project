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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

// Adjustable set height off from starting position
#define dz 0.5
#define PEN_LEN 0.35
#define FREQ 4.5 //sqrt(9.81/PEN_LEN)
#define VEL_THRESH 3.0

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
volatile double qx;
volatile double qy;
volatile double qz;

volatile double lbx = 0;
volatile double lby = 0;
volatile double lbz = 0;

volatile double vx = 0;
volatile double vy = 0;
volatile double vz = 0;
volatile double t = 0;
volatile double zmax = 0;

ros::Time lstamp;
ros::Time lastPredictTime;
ros::Publisher ball_vel;
geometry_msgs::TwistStamped vel_msg;
ros::Duration dt;
double alpha = 0.1;
bool acceptBallSetpoint = false;
bool thisIsTheMoney = false;
double zThreshold;

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

void ballCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {

    // Define transform and its broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    if (acceptBallSetpoint) {
        if( ros::Time::now() - lastPredictTime < ros::Duration(0.5) ) {
            // Updtate time increment
            dt = msg->header.stamp - lstamp;
            lstamp = msg->header.stamp;

            if( dt.toSec() > 0 ) {
                // Apply exponentially moving average filter to velocity approximations
                vx = alpha*( msg->point.x - lbx )/dt.toSec() + (1-alpha)*vx;
                vy = alpha*( msg->point.y - lby )/dt.toSec() + (1-alpha)*vy;
                vz = alpha*( msg->point.z - lbz )/dt.toSec() + (1-alpha)*vz;

                vel_msg.header.stamp = msg->header.stamp;
                vel_msg.twist.linear.x = vx;
                vel_msg.twist.linear.y = vy;
                vel_msg.twist.linear.z = vz;
                ball_vel.publish(vel_msg);

                if( fabs(vx) < VEL_THRESH && fabs(vy) < VEL_THRESH && fabs(vz) < VEL_THRESH ) {
                    if( swingingUp ) {
                        // Calculate zmax based on energy in pendulum in body frame
                        zmax = (msg->point.z)  + 0.5*vy*vy*0.101936799 + 0.5*vz*vz*0.101936799;
                        ROS_INFO("Zmax(%f) Zball(%f) QZ(%f) VZ(%f) dt(%f)",zmax-qz,msg->point.z,qz,vz,dt.toSec());

                        if( ~isnan(zmax) && std::isfinite(zmax) ) {
                            if( zmax > qz+zThreshold && vz > 0 && msg->point.z < (qz-0.27) ) {
                                thisIsTheMoney = true;
                            }
                        }

                        if( thisIsTheMoney && msg->point.z > (qz - 0.27) && msg->point.z < (qz - 0.1) ) {
                            swingingUp = false;
                            lastPredictTime = ros::Time::now() - ros::Duration(5);
                            vx = 0;
                            vy = 0;
                            vz = 0;
                        }

                    } else {
                        // Calculated closed form value of time at which ball will cross the
                        // quadcopter's z plane
                        if( msg->point.z < qz ) {
                            xD = msg->point.x;
                            yD = msg->point.y;
                        } else {
                            t = 0.102*(vz + sqrt( vz*vz + 19.62*msg->point.z - 19.62*qz));

                            if ( ~isnan(t) && std::isfinite(t) ) {
                                // Solve for the x,y position at the time of impact
                                double x = msg->point.x + vx*t;
                                double y = msg->point.y + vy*t;
                                ROS_INFO("Tracking X(%f) Y(%f) VX(%f) VY(%f)",x,y,vx,vy);
                                if( x < 2 && x > -2 && y < 1.5 && y > -1 ) {

                                    xD = x;
                                    yD = y;
                                }
                            }
                        }
                    }
                }
                lastPredictTime = ros::Time::now();
            }
        } else {
            lstamp = msg->header.stamp;
            lastPredictTime = ros::Time::now();
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
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    ros::Subscriber quad_sub = n.subscribe<geometry_msgs::PointStamped>("/crazyflie/external_position",1,positionCallback);
    ros::Subscriber ball_sub = n.subscribe<geometry_msgs::PointStamped>("/ball/position",1,ballCallback);
    ros::Publisher  waypoint_pub = n.advertise<geometry_msgs::PointStamped>("/crazyflie/cmd_waypoint",1);
    ros::Publisher  target_pub = n.advertise<geometry_msgs::PointStamped>("/target_pos",1);
    ros::Subscriber ready_sub = n.subscribe<std_msgs::UInt8>("/crazyflie/ready",1,readyCallback);
    ros::Subscriber ext_ready = n.subscribe<std_msgs::UInt8>("/external_position/ready",1,extPosReadyCallback);
    ros::Publisher  xyKp_pub  = n.advertise<std_msgs::Float32>("/crazyflie/xyKp",1);
    ros::Publisher  hov_pub   = n.advertise<crazyflie_driver::HoverStamped>("/crazyflie/cmd_hover",1);
    ball_vel = n.advertise<geometry_msgs::TwistStamped>("/crazyflie/velocity",1);

    n_private.param<double>("zThreshold", zThreshold, 0.05);

    std_msgs::Float32 xyKp_msg;
    geometry_msgs::PointStamped waypoint_msg;
    crazyflie_driver::HoverStamped hov_msg;

    ros::Rate slow_loop(5);
    ros::Rate fast_loop(200);

    lstamp = ros::Time::now();

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
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "target";

    double targZ;

    ROS_INFO("Taking off...");
    for( int i = 0; i < 30; i ++ ) {
        hov_msg.vx = 0;
        hov_msg.vy = 0;
        hov_msg.yawrate = 0;
        targZ = zS + dz*(1.0 - (30-(float)i)/30);
        hov_msg.zDistance = targZ;
        hov_pub.publish(hov_msg);
        ros::spinOnce();
        ros::spinOnce();
        slow_loop.sleep();
    }

    ROS_INFO("Sending static setpoint...");
    xS = qx;
    yS = qy;
    waypoint_msg.point.x = xS;
    waypoint_msg.point.y = yS;
    waypoint_msg.point.z = zS + dz;
    // Hover at desired point for 3 seconds to allow settling of ball movement
    for( int i = 0; i < 30; i ++ ) {
        waypoint_pub.publish(waypoint_msg);
        ros::spinOnce();
        slow_loop.sleep();
    }

    // Update the xy position P gain for faster response
    xyKp_msg.data = 4.0;
    xyKp_pub.publish(xyKp_msg);

    lstamp = ros::Time::now();
    lastPredictTime = ros::Time::now();
    // Allow the ball setpoint tracker to run
    acceptBallSetpoint = true;
    // Set inital desired position (x and y updated async in ballCallback)
    xD = xS;
    yD = yS;
    zD = zS + dz;
    geometry_msgs::PointStamped msg;
    msg.header.frame_id ="target";
    // Set the copter to swingup mode (changed in ballCallback)
    swingingUp = true;

    ros::Duration ct;
    ros::Time startTime = ros::Time::now();

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
            waypoint_msg.point.y = yS + 0.5*sin(FREQ*ct.toSec());    // 28cm
            waypoint_msg.point.z = zD;

            // Update target transform
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = qx;
            transformStamped.transform.translation.y = qy + sqrt(PEN_LEN*PEN_LEN - (qz-zmax)*(qz-zmax));
            transformStamped.transform.translation.z = qz + zmax;
            transformStamped.transform.rotation.w = 1;
            // Update target point message
            msg.point.x = qx;
            msg.point.y = sqrt(PEN_LEN*PEN_LEN - zmax*zmax);
            msg.point.z = zmax;

        } else {
            // Track trajectory from forward balistic model
            waypoint_msg.header.stamp = ros::Time::now();
            waypoint_msg.point.x = xD;
            waypoint_msg.point.y = yD;
            // Dip in z to aid in catching
            waypoint_msg.point.z = zD-0.3;

            // Update target transform
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = xD;
            transformStamped.transform.translation.y = yD;
            transformStamped.transform.translation.z = zD-0.3;
            transformStamped.transform.rotation.w = 1;
            // Update target point message
            msg.point.x = xD;
            msg.point.y = yD;
            msg.point.z = zD;
        }

        // Send waypoint to quad
        waypoint_pub.publish(waypoint_msg);
        // Publish target position message
        target_pub.publish(msg);
        // Send off transform
        if( ~isnan(transformStamped.transform.translation.y) && std::isfinite(transformStamped.transform.translation.y) ) {
            br.sendTransform(transformStamped);
        }

        ros::spinOnce();
        fast_loop.sleep();
    }

}
