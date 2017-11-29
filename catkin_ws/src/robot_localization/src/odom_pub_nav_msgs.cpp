/******************************************************************************
    * odom_pub_nav_msgs.cpp -- wheel odometry nav_msgs publisher              *
    * June 22, 2017                                                           *
    *                                                                         *
    * Carnegie Mellon Univesity                                               *
    *                                                                         *  
    * Author:   Addepalli, Shalini                                            *
    *           Papincak, Lawrence                                            *
    *                                                                         *
    * Purpose:  Converts encoder messages from motors into odometry messages  *
    *           to be interpreted by the EKF node.                            *
    *                                                                         *
    * Usage:                                                                  *
    **************************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <msgs/Motor_Encoder_Values.h>
#include <iostream>

ros::Publisher odom_pub;
nav_msgs::Odometry odom_data;

double wheel_diam;	// Drive wheel diameter
double gear_ratio;	// Motor gear ratio
double ext_ratio;	// External gear ratio
double enc_ppr;		// Encoder pulses per revolution
double conv_factor; // Conversion factor from counts to distance
double pi;

void encCallback(const msgs::Motor_Encoder_Values::ConstPtr& msg) {
	
	odom_data.header = msg->header;
	odom_data.header.frame_id = "odom";
	odom_data.child_frame_id = "base_link";
	odom_data.pose.pose.position.x = msg->drive1_counts * conv_factor;
	odom_data.pose.pose.position.y = 0;
	odom_data.pose.pose.position.z = 0;
	odom_pub.publish(odom_data);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "odometry_publisher_nav_msgs");
	ros::NodeHandle n;
	
	n.param<double>("wheel_diam",       wheel_diam, 0.1016);
	n.param<double>("drive_gear_ratio", gear_ratio, 410.77);
	n.param<double>("drive_ext_ratio",  ext_ratio, 1);
	n.param<double>("enc_ppr",          enc_ppr, 2000);
	n.param<double>("pi",               pi,3.14159);
	
	conv_factor = (pi * wheel_diam)/(gear_ratio*ext_ratio*enc_ppr);

	odom_pub = n.advertise<nav_msgs::Odometry>("/odom_wheel",50);
	ros::Subscriber sub = n.subscribe("/motor/encoders", 1000, encCallback);
	
	ros::spin();

	return 0;
}

	
	
