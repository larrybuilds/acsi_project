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
#include <std_msgs/UInt8.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <inttypes.h>

#define ALPHA 0.3

int rbID;
int rbIndex;
int ballIndex;
bool ballRelative;
float ball_roi;

double rx, ry, rz, bx, by, bz;

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
    } else {
        //ROS_WARN("No rigid body found with ID: %d", rbID);
    }
}

void unidentifiedPointCallback(const optitrack::UnidentifiedPointArray::ConstPtr& msg) {
    ballIndex = -1;
    // ROS_INFO("Found %d points", (int)msg->upoints.size());
    for( int i = 0; i < msg->upoints.size(); i++ ) {
        if( ballRelative ) {
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
        } else {
            if( msg->upoints[i].x < 1.5 &&
                msg->upoints[i].x > -1.0 &&
                msg->upoints[i].y < 2.5 &&
                msg->upoints[i].y > 0.0 &&
                msg->upoints[i].z < 2.0 &&
                msg->upoints[i].z > -2.0)  {

                ballIndex = i;
                break;
            }
        }
    }

    if( ballIndex >= 0 ) {
        bx = msg->upoints[ballIndex].z;
        by = msg->upoints[ballIndex].x;
        bz = msg->upoints[ballIndex].y;
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

    n_private.param<std::string>("rigid_topic",      rigid_topic,  "/optitrack/rigid_bodies");
    n_private.param<std::string>("upoint_topic",     upoint_topic, "/optitrack/upoints");
    n_private.param<std::string>("ball_pose_topic",  ball_topic,   "/ball/position");

    n_private.param<int>("rigid_id", rbID, 1);
    n_private.param<bool>("ballRelative", ballRelative, false);
    n_private.param<float>("ball_roi", ball_roi, 0.4);

    ros::Publisher  ball_pub = n.advertise<geometry_msgs::PointStamped>(ball_topic, 1);
    ros::Publisher  ext_pose = n.advertise<geometry_msgs::PointStamped>("/crazyflie/external_position",1);
    ros::Publisher ext_pose_ready = n.advertise<std_msgs::UInt8>("/external_position/ready",1);
    ros::Subscriber rig_sub  = n.subscribe<optitrack::RigidBodyArray>(rigid_topic,1,rigidBodyCallback);
    ros::Subscriber upt_sub  = n.subscribe<optitrack::UnidentifiedPointArray>(upoint_topic,1,unidentifiedPointCallback);

    // Define transform and its broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    ros::Rate loop_rate(200);

    rbIndex = -1;
    ballIndex = -1;
    bool firstExternalPosition = true;

    geometry_msgs::PointStamped pnt_msg;
    geometry_msgs::PointStamped ball_msg;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while( ros::ok() ) {

        if( ballIndex >= 0 ) {

            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "ball";
            // Set relative position to zero
            transformStamped.transform.translation.x = bx;
            transformStamped.transform.translation.y = by;
            transformStamped.transform.translation.z = bz;
            transformStamped.transform.rotation.w = 1;
            // Send off transform
            br.sendTransform(transformStamped);

            ball_msg.header.stamp = ros::Time::now();
            ball_msg.header.frame_id = "ball";
            ball_msg.point.x = bx;
            ball_msg.point.y = by;
            ball_msg.point.z = bz;
            // ROS_INFO("Ball X(%f) Y(%f) Z(%f)...", bx, by, bz);
            ball_pub.publish(ball_msg);
        }

        if( rbIndex >= 0 ) {
            if( firstExternalPosition ) {
                firstExternalPosition = false;
                std_msgs::UInt8 fsp_msg;
                ext_pose_ready.publish(fsp_msg);
            }
            pnt_msg.header.stamp = ros::Time::now();
            pnt_msg.point.x = rx;
            pnt_msg.point.y = ry;
            pnt_msg.point.z = rz;

            ext_pose.publish(pnt_msg);
        }

        loop_rate.sleep();
    }

    spinner.stop();
}
