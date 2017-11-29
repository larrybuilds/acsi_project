#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <msgs/Motor_Encoder_Values.h>
#include <iostream>
#include <boost/array.hpp>

nav_msgs::Odometry odomMsg;
ros::Publisher odom_pub;
typedef boost::array<double, 36ul> cov;
cov covar;

void encCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	
	odomMsg.header = msg->header;
	odomMsg.header.frame_id = "odom";
	odomMsg.child_frame_id = "base_link";
	odomMsg.pose.pose.position.x = msg->pose.pose.position.x;
	odomMsg.pose.pose.position.y = 0;
	odomMsg.pose.pose.position.z = 0;
	odomMsg.pose.covariance = covar;
	odom_pub.publish(odomMsg);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_repub");
	ros::NodeHandle n;

	int loc = 0;
	int count = 0; 
	for(int i = 0; i < 6; i++ ) {
		for(int j = 0; j < 6; j ++ ) {
			if( j == loc ) {
				covar[count] = 1;
			} else {
				covar[count] = 0;
			}
			count++;
		}
		loc++;
	}

	odom_pub = n.advertise<nav_msgs::Odometry>("/odom/idler1/cov",50);
	ros::Subscriber sub = n.subscribe("/odom/idler1", 1000, encCallback);
	
	ros::spin();

	return 0;
}
