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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

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
        //ROS_WARN("No rigid body found with ID: %d", rbID);
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
        //ROS_WARN("No suspended ball found!");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    std::string rigid_topic;
    std::string upoint_topic;
    std::string ball_topic;
    std::string quad_topic;

    n_private.param<std::string>("rigid_topic",  rigid_topic,  "/optitrack/rigid_bodies");
    n_private.param<std::string>("upoint_topic", upoint_topic, "/optitrack/upoints");
    n_private.param<std::string>("ball_pose_topic",  ball_topic,  "/ball/pose");
    n_private.param<std::string>("quad_pose_topic",  quad_topic,  "/quad/pose");

    n_private.param<int>("rigid_id", rbID, 0);
    n_private.param<float>("ball_roi", ball_roi, 0.25);

    ros::Publisher  ball_pub = n.advertise<nav_msgs::Odometry>(ball_topic, 1);
    ros::Publisher  quad_pub = n.advertise<nav_msgs::Odometry>(quad_topic, 1);
    ros::Subscriber rig_sub = n.subscribe<optitrack::RigidBodyArray>(rigid_topic,1,rigidBodyCallback);
    ros::Subscriber upt_sub = n.subscribe<optitrack::UnidentifiedPointArray>(upoint_topic,1,unidentifiedPointCallback);

    ros::Rate loop_rate(400);

    nav_msgs::Odometry ball_pose_msg;
    nav_msgs::Odometry quad_pose_msg;

    quad_pose_msg.header.frame_id = "odom";
    ball_pose_msg.header.frame_id = "base_link";
    quad_pose_msg.child_frame_id = "base_link";
    ball_pose_msg.child_frame_id = "ball";

    while( ros::ok() ) {

        if( ballIndex >= 0 ) {
            ball_pose_msg.header.stamp = ros::Time::now();

            ball_pose_msg.pose.pose.position.x = bx;
            ball_pose_msg.pose.pose.position.y = by;
            ball_pose_msg.pose.pose.position.z = bz;

            ball_pub.publish(ball_pose_msg);
        }

        if( rbIndex >= 0 ) {
            quad_pose_msg.header.stamp = ros::Time::now();

            quad_pose_msg.pose.pose.position.x = rx;
            quad_pose_msg.pose.pose.position.y = ry;
            quad_pose_msg.pose.pose.position.z = rz;

            quad_pose_msg.pose.pose.orientation.w = qw;
            quad_pose_msg.pose.pose.orientation.x = qx;
            quad_pose_msg.pose.pose.orientation.y = qy;
            quad_pose_msg.pose.pose.orientation.z = qz;

            quad_pub.publish(quad_pose_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}
