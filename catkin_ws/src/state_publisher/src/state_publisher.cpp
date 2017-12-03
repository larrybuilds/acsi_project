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
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

#define ALPHA 0.3

int rbID;
int rbIndex;
int ballIndex;
float ball_roi;
float rx, ry, rz;   // In ROS world coords
float drx, dry, drz;
float droll, dpitch, dyaw;
float qw, qx, qy, qz;
float bx, by, bz;   // In ROS world coords

float last_roll, last_pitch, last_yaw;
float last_rx, last_ry, last_rz;

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
    ros::Publisher  quad_state = n.advertise<nav_msgs::Odometry>("/odometry/filtered", 1);
    ros::Publisher  ext_pose = n.advertise<geometry_msgs::PointStamped>("/crazyflie/external_position",1);
    ros::Subscriber rig_sub = n.subscribe<optitrack::RigidBodyArray>(rigid_topic,1,rigidBodyCallback);
    ros::Subscriber upt_sub = n.subscribe<optitrack::UnidentifiedPointArray>(upoint_topic,1,unidentifiedPointCallback);

    ros::Rate loop_rate(1);

    nav_msgs::Odometry ball_pose_msg;
    nav_msgs::Odometry quad_pose_msg;
    nav_msgs::Odometry quad_state_msg;

    quad_pose_msg.header.frame_id = "odom";
    ball_pose_msg.header.frame_id = "base_link";
    quad_pose_msg.child_frame_id = "base_link";
    ball_pose_msg.child_frame_id = "ball";

    quad_state_msg.header.frame_id = "odom";
    quad_state_msg.child_frame_id = "base_link";

    ros::Duration dt;
    ros::Time lastTime = ros::Time::now();

    last_roll = 0;
    last_pitch = 0;
    last_yaw = 0;
    last_rx = 0;
    last_ry = 0;
    last_rz = 0;
    drx = 0;
    dry = 0;
    drz = 0;
    droll = 0;
    dpitch = 0;
    dyaw = 0;

    rbIndex = -1;
    ballIndex = -1;

    ros::Duration(5).sleep();

    while( ros::ok() ) {

        dt = ros::Time::now() - lastTime;
        lastTime = ros::Time::now();

        if( ballIndex >= 0 ) {
            ball_pose_msg.header.stamp = ros::Time::now();

            ball_pose_msg.pose.pose.position.x = bx;
            ball_pose_msg.pose.pose.position.y = by;
            ball_pose_msg.pose.pose.position.z = bz;

            ball_pub.publish(ball_pose_msg);
        }

        if( rbIndex >= 0 ) {
            // quad_pose_msg.header.stamp = ros::Time::now();
            //
            // quad_pose_msg.pose.pose.position.x = rx;
            // quad_pose_msg.pose.pose.position.y = ry;
            // quad_pose_msg.pose.pose.position.z = rz;
            //
            // quad_pose_msg.pose.pose.orientation.w = qw;
            // quad_pose_msg.pose.pose.orientation.x = qx;
            // quad_pose_msg.pose.pose.orientation.y = qy;
            // quad_pose_msg.pose.pose.orientation.z = qz;
            //
            // quad_pub.publish(quad_pose_msg);



            // quad_state_msg.header.stamp = ros::Time::now();
            //
            // quad_state_msg.pose.pose.position.x = rx;
            // quad_state_msg.pose.pose.position.y = ry;
            // quad_state_msg.pose.pose.position.z = rz;
            // quad_state_msg.pose.pose.orientation.w = qw;
            // quad_state_msg.pose.pose.orientation.x = qx;
            // quad_state_msg.pose.pose.orientation.y = qy;
            // quad_state_msg.pose.pose.orientation.z = qz;
            //
            // drx = ALPHA*((rx - last_rx)/dt.toSec()); + (1-ALPHA)*drx;
            // dry = ALPHA*((ry - last_ry)/dt.toSec()) + (1-ALPHA)*dry;
            // drz = ALPHA*((rz - last_rz)/dt.toSec()) + (1-ALPHA)*drz;
            //
            // tfScalar roll, pitch, yaw;
            // tf::Matrix3x3(
            //     tf::Quaternion(
            //         qx,
            //         qy,
            //         qz,
            //         qw
            //     )).getRPY(roll, pitch, yaw);
            // droll  = ALPHA*((roll-last_roll)/dt.toSec())   + (1-ALPHA)*droll;
            // dpitch = ALPHA*((pitch-last_pitch)/dt.toSec()) + (1-ALPHA)*dpitch;
            // dyaw   = ALPHA*((yaw-last_yaw)/dt.toSec())     + (1-ALPHA)*dyaw;
            // //ROS_INFO("dr: %f   dp: %f   dy: %f", roll-last_roll, pitch-last_pitch, yaw-last_yaw);
            // last_roll = roll;
            // last_pitch = pitch;
            // last_yaw = yaw;
            //
            // last_rx = rx;
            // last_ry = ry;
            // last_rz = rz;
            //
            // quad_state_msg.twist.twist.linear.x = drx;
            // quad_state_msg.twist.twist.linear.y = dry;
            // quad_state_msg.twist.twist.linear.z = drz;
            // quad_state_msg.twist.twist.angular.x = droll;
            // quad_state_msg.twist.twist.angular.y = dpitch;
            // quad_state_msg.twist.twist.angular.z = dyaw;
            //
            // quad_state.publish(quad_state_msg);

            geometry_msgs::PointStamped pnt_msg;
            pnt_msg.header.stamp = ros::Time::now();
            pnt_msg.point.x = rx;
            pnt_msg.point.y = ry;
            pnt_msg.point.z = rz;

            ext_pose.publish(pnt_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}
