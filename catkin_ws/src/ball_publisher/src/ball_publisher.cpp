/******************************************************************************
    * ball_publisher.cpp -- Publishes the position of the ball suspended from *
                            the quad rotor rx filtering the set of all points *
                            in the optitrack field-of-view                    *
    * Carnegie Mellon Univesity                                               *
    *                                                                         *
    * Author:   Papincak, Lawrence                                            *
    *                                                                         *
    **************************************************************************/

#include "ros/ros.h"
#include "optitrack/UnidentifiedPointArray.h"
#include "optitrack/RigidBodyArray.h"
#include "ball_publisher/FlyingBilboPose.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

int rbID;
int rbIndex;
int ballIndex;
float ball_roi;
float rx, ry, rz;   // In ROS world coords
float qw, qx, qy, qz;
float bx, by, bz;   // In ROS world coords

void rigidBodyCallback(const optitrack::RigidBodyArray::ConstPtr& msg) {
    rbIndex = -1;
    for(int i = 0; i < msg->bodies.size(); i ++) {
        if( msg->bodies[i].id == rbID ) {
            rbIndex = i;
            break;
        }
    }

    if( rbIndex >= 0 ) {
        rx = msg->bodies[rbIndex].pose.position.x;
        ry = msg->bodies[rbIndex].pose.position.y;
        rz = msg->bodies[rbIndex].pose.position.z;

        qw = msg->bodies[rbIndex].pose.orientation.w;
        qx = msg->bodies[rbIndex].pose.orientation.x;
        qy = msg->bodies[rbIndex].pose.orientation.y;
        qz = msg->bodies[rbIndex].pose.orientation.z;

    } else {
        ROS_WARN("No rigid body found with ID: %d", rbID);
    }
}

void unidentifiedPointCallback(const optitrack::UnidentifiedPointArray::ConstPtr& msg) {
    ballIndex = -1;
    // ROS_INFO("Found %d points", (int)msg->upoints.size());
    for( int i = 0; i < msg->upoints.size(); i++ ) {
        if( msg->upoints[i].x < (ry + ball_roi) &&
            msg->upoints[i].x > (ry - ball_roi) &&
            msg->upoints[i].y < (rz + ball_roi) &&
            msg->upoints[i].y > (rz - ball_roi) &&
            msg->upoints[i].z < (rx + ball_roi) &&
            msg->upoints[i].z > (rx - ball_roi))  {

            // ROS_INFO("X %f to (%f,%f)",msg->upoints[i].x,ry + ball_roi,ry - ball_roi);
            // ROS_INFO("Y %f to (%f,%f)",msg->upoints[i].y,rz + ball_roi,ry - ball_roi);
            // ROS_INFO("Z %f to (%f,%f)",msg->upoints[i].z,rx + ball_roi,ry - ball_roi);

            ballIndex = i;
            // ROS_INFO("Found ball at point index #%d", i);
            break;
        }
    }

    if( ballIndex >= 0 ) {
        bx = msg->upoints[ballIndex].z;
        by = msg->upoints[ballIndex].x;
        bz = msg->upoints[ballIndex].y;
    } else {
        ROS_WARN("No suspended ball found!");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ball_publisher");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    std::string rigid_topic;
    std::string upoint_topic;
    std::string bilbo_topic;

    n_private.param<std::string>("rigid_topic",  rigid_topic,  "/optitrack/rigid_bodies");
    n_private.param<std::string>("upoint_topic", upoint_topic, "/optitrack/upoints");
    n_private.param<std::string>("bilbo_topic",  bilbo_topic,  "/bilbo/pose");
    n_private.param<int>("rigid_id", rbID, 0);
    n_private.param<float>("ball_roi", ball_roi, 0.25);

    ros::Publisher  bilbo_pub = n.advertise<ball_publisher::FlyingBilboPose>(bilbo_topic, 100);
    ros::Subscriber rig_sub = n.subscribe<optitrack::RigidBodyArray>(rigid_topic,100,rigidBodyCallback);
    ros::Subscriber upt_sub = n.subscribe<optitrack::UnidentifiedPointArray>(upoint_topic,100,unidentifiedPointCallback);

    ros::Rate loop_rate(200);

    ball_publisher::FlyingBilboPose pose_msg;

    while( ros::ok() ) {
        if( rbIndex >= 0 ) {
            pose_msg.validQuad = true;
        } else {
            pose_msg.validQuad = false;
        }

        if( ballIndex >= 0 ) {
            pose_msg.validBall = true;
        } else {
            pose_msg.validBall = false;
        }

        pose_msg.ball.x = bx;
        pose_msg.ball.y = by;
        pose_msg.ball.z = bz;

        pose_msg.quad.position.x = rx;
        pose_msg.quad.position.y = ry;
        pose_msg.quad.position.z = rz;

        pose_msg.quad.orientation.w = qw;
        pose_msg.quad.orientation.x = qx;
        pose_msg.quad.orientation.y = qy;
        pose_msg.quad.orientation.z = qz;

        bilbo_pub.publish(pose_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }
}
