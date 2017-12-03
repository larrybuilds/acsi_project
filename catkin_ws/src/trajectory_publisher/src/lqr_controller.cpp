/******************************************************************************
    * lqr_controller.cpp -- Computes the control necessary to achieve desired *
    *                   trajectory
    * Carnegie Mellon Univesity                                               *
    *                                                                         *
    * Author:   Papincak, Lawrence                                            *
    *                                                                         *
    * Usage:                                                                  *
    **************************************************************************/

#include <ros/ros.h>
#include <crazyflie_driver/TorqueThrustStamped.h>
#include "crazyflie_driver/HoverStamped.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include "trajectory_publisher/lqr.hpp"

#include <iostream>

class LQR
{
public:

    LQR(const std::string& worldFrame,
        const std::string& frame,
        double freq,
        const ros::NodeHandle& n)
        : _tx(
            get(n,"/Gains/tx/kx"),
            get(n,"/Gains/tx/ky"),
            get(n,"/Gains/tx/kz"),
            get(n,"/Gains/tx/kroll"),
            get(n,"/Gains/tx/kpitch"),
            get(n,"/Gains/tx/kyaw"),
            get(n,"/Gains/tx/kdx"),
            get(n,"/Gains/tx/kdy"),
            get(n,"/Gains/tx/kdz"),
            get(n,"/Gains/tx/kdroll"),
            get(n,"/Gains/tx/kdpitch"),
            get(n,"/Gains/tx/kdyaw"))
        , _ty(
            get(n,"/Gains/ty/kx"),
            get(n,"/Gains/ty/ky"),
            get(n,"/Gains/ty/kz"),
            get(n,"/Gains/ty/kroll"),
            get(n,"/Gains/ty/kpitch"),
            get(n,"/Gains/ty/kyaw"),
            get(n,"/Gains/ty/kdx"),
            get(n,"/Gains/ty/kdy"),
            get(n,"/Gains/ty/kdz"),
            get(n,"/Gains/ty/kdroll"),
            get(n,"/Gains/ty/kdpitch"),
            get(n,"/Gains/ty/kdyaw"))
        , _tz(
            get(n,"/Gains/tz/kx"),
            get(n,"/Gains/tz/ky"),
            get(n,"/Gains/tz/kz"),
            get(n,"/Gains/tz/kroll"),
            get(n,"/Gains/tz/kpitch"),
            get(n,"/Gains/tz/kyaw"),
            get(n,"/Gains/tz/kdx"),
            get(n,"/Gains/tz/kdy"),
            get(n,"/Gains/tz/kdz"),
            get(n,"/Gains/tz/kdroll"),
            get(n,"/Gains/tz/kdpitch"),
            get(n,"/Gains/tz/kdyaw"))
        , _T(
            get(n,"/Gains/T/kx"),
            get(n,"/Gains/T/ky"),
            get(n,"/Gains/T/kz"),
            get(n,"/Gains/T/kroll"),
            get(n,"/Gains/T/kpitch"),
            get(n,"/Gains/T/kyaw"),
            get(n,"/Gains/T/kdx"),
            get(n,"/Gains/T/kdy"),
            get(n,"/Gains/T/kdz"),
            get(n,"/Gains/T/kdroll"),
            get(n,"/Gains/T/kdpitch"),
            get(n,"/Gains/T/kdyaw"))
        , _freq(freq)
        , _state(waiting)
        , _worldFrame(worldFrame)
        , _frame(frame)
        , _startZ(0)
        , _thrust(0)
    {
        ros::NodeHandle nh;

        // Wait for the copter to world transform to come online
        if( !_listener.waitForTransform(_worldFrame, _frame, ros::Time(0), ros::Duration(10.0))) {
            ROS_WARN("Failed to recieve transform from world frame to quadcopter frame");
        } else {
            ROS_INFO("Found transform");
        }

        _att_setpoint = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        _tt_setpoint  = nh.advertise<crazyflie_driver::TorqueThrustStamped>("cmd_thrust_torque",1);
        _hov_setpoint = nh.advertise<crazyflie_driver::HoverStamped>("cmd_hover",1);

        _serviceTakeoff = nh.advertiseService("takeoff", &LQR::takeoff, this);
        _serviceLand    = nh.advertiseService("land", &LQR::land, this);
        _waypoint_sub   = nh.subscribe<nav_msgs::Odometry>("/waypoint", 1, &LQR::updateWaypointCallback, this);
        _state_sub      = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &LQR::updateStateCallback, this);

        // Initialize matrix sizes
        _K.resize(4,12);
        _X.resize(12);
        _Xd.resize(12);
        _U.resize(4);
        _Unom.resize(4);

        // Load in LQR gains
        for(int i = 0; i < NUM_STATES; i++ ) {
            _K(THRUST_STATE,i)   = _T.gains[i];
            _K(TORQUE_X_STATE,i) = _tx.gains[i];
            _K(TORQUE_Y_STATE,i) = _ty.gains[i];
            _K(TORQUE_Z_STATE,i) = _tz.gains[i];
        }

        std::cout << "K" << std::endl << _K << std::endl;

        _Unom(0) = 0.037*9.81;
        _Unom(1) = 0;
        _Unom(2) = 0;
        _Unom(3) = 0;
    }

    void spin() {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/_freq), &LQR::stateCallback, this);
        ros::spin();
    }

private:

    /* @brief Updates internal waypoint

     */
    void updateWaypointCallback( const nav_msgs::Odometry::ConstPtr& msg ) {

        tfScalar roll, pitch, yaw;
        tf::Matrix3x3(
            tf::Quaternion(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            )).getRPY(roll, pitch, yaw);

        _Xd[X_STATE]         = msg->pose.pose.position.x;
        _Xd[Y_STATE]         = msg->pose.pose.position.y;
        _Xd[Z_STATE]         = msg->pose.pose.position.z;
        _Xd[X_DOT_STATE]     = msg->twist.twist.linear.x;
        _Xd[Y_DOT_STATE]     = msg->twist.twist.linear.y;
        _Xd[Z_DOT_STATE]     = msg->twist.twist.linear.z;
        _Xd[ROLL_STATE]      = roll;
        _Xd[PITCH_STATE]     = pitch;
        _Xd[YAW_STATE]       = yaw;
        _Xd[ROLL_DOT_STATE]  = msg->twist.twist.angular.x;
        _Xd[PITCH_DOT_STATE] = msg->twist.twist.angular.y;
        _Xd[YAW_DOT_STATE]   = msg->twist.twist.angular.z;
    }

    /* @brief Updates the internal state estimate

     */
    void updateStateCallback( const nav_msgs::Odometry::ConstPtr& msg) {
        tfScalar roll, pitch, yaw;
        tf::Matrix3x3(
            tf::Quaternion(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            )).getRPY(roll, pitch, yaw);

        _X[X_STATE]         = msg->pose.pose.position.x;
        _X[Y_STATE]         = msg->pose.pose.position.y;
        _X[Z_STATE]         = msg->pose.pose.position.z;
        _X[X_DOT_STATE]     = msg->twist.twist.linear.x;
        _X[Y_DOT_STATE]     = msg->twist.twist.linear.y;
        _X[Z_DOT_STATE]     = msg->twist.twist.linear.z;
        _X[ROLL_STATE]      = roll;
        _X[PITCH_STATE]     = pitch;
        _X[YAW_STATE]       = yaw;
        _X[ROLL_DOT_STATE]  = msg->twist.twist.angular.x;
        _X[PITCH_DOT_STATE] = msg->twist.twist.angular.y;
        _X[YAW_DOT_STATE]   = msg->twist.twist.angular.z;
    }

    /* @brief Callback for takeoff service

     */
    bool takeoff( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Takeoff requested!");
        _state = takingoff;

        tf::StampedTransform transform;
        _listener.lookupTransform(_worldFrame, _frame, ros::Time(0), transform);
        _startZ = transform.getOrigin().z();
        return true;
    }

    /* @brief Callback for landing service

     */
    bool land( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Landing requested!");
        _thrust = 50000;
        _state = landing;
        return true;
    }

    /* @brief Callback for timer that contains state machine

     */

    void stateCallback(const ros::TimerEvent& e) {

        // Get differental time incrememtn between timer executition
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(_state) {

            // Get up off the ground to get away from ground effect
            case takingoff: {

                tf::StampedTransform transform;
                _listener.lookupTransform(_worldFrame, _frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > _startZ + 0.05 || _thrust > 40000) {
                    _state = automatic;
                    _thrust = 0;
                }
                else {
                    _thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = _thrust;
                    _att_setpoint.publish(msg);
                }
                break;
            }// case takingoff:

            // Land slowly back down on the ground
            case landing: {
                tf::StampedTransform transform;
                _listener.lookupTransform(_worldFrame, _frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= _startZ + 0.05) {
                    _state = waiting;
                    geometry_msgs::Twist msg;
                    _att_setpoint.publish(msg);
                } else {
                    _thrust -= 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = _thrust;
                    _att_setpoint.publish(msg);
                }
                break;
            }// case landing:

            // Run controller and automatically track waypoints
            case automatic: {
                // // Compute error at current timestep
                // _E = (_Xd - _X);
                // //std::cout << "E" << std::endl << _E << std::endl;
                // // Compute the current control
                // _U = _K*_E + _Unom;
                //std::cout << "U" << std::endl << _U << std::endl;

                // crazyflie_driver::TorqueThrustStamped tt_msg;
                // // Load thrust-torque setpoint message with control
                // tt_msg.header.stamp = ros::Time::now();
                // ROS_INFO("Thrust: %f", _U(THRUST_STATE));
                // tt_msg.thrust = (((float)_U(THRUST_STATE))/9.81/0.06)*65536;
                // tt_msg.tx     = _U(TORQUE_X_STATE);
                // tt_msg.ty     = _U(TORQUE_Y_STATE);
                // tt_msg.tz     = _U(TORQUE_Z_STATE);
                // // Publish message
                // _tt_setpoint.publish(tt_msg);

                // geometry_msgs::Twist att_msg;
                // att_msg.linear.x = 0;
                // att_msg.linear.y = 0;
                // att_msg.linear.z = 41000;//(((float)_U(THRUST_STATE))/9.81/0.06)*65536;
                // att_msg.angular.z = 0;
                // _att_setpoint.publish(att_msg);

                crazyflie_driver::HoverStamped h_msg;

                h_msg.header.stamp = ros::Time::now();
                h_msg.vx = 0;
                h_msg.vy = 0;
                h_msg.yawrate = 0;
                h_msg.zDistance = 1.0;

                _hov_setpoint.publish(h_msg);

                break;
            }// case automatic:

            // Send zero setpoint to maintain position until takeoff
            case waiting: {
                crazyflie_driver::TorqueThrustStamped tt_msg;
                _tt_setpoint.publish(tt_msg);
                break;
            }// case waiting:

        } // swtich(_state)
    }

private:
    enum State {waiting, automatic, takingoff, landing};

    Eigen::MatrixXd _K;     // LQR gain matrix
    Eigen::VectorXd _X;     // State vector
    Eigen::VectorXd _Xd;    // Desired state
    Eigen::VectorXd _E;     // Error vector
    Eigen::VectorXd _U;     // Control vector
    Eigen::VectorXd _Unom;  // Nominal control vector

    LQR_gains _tx;
    LQR_gains _ty;
    LQR_gains _tz;
    LQR_gains _T;

    State _state;

    nav_msgs::Odometry _waypoint;

    std::string _worldFrame;
    std::string _frame;

    tf::TransformListener _listener;
    ros::ServiceServer _serviceTakeoff;
    ros::ServiceServer _serviceLand;

    ros::Publisher _att_setpoint;
    ros::Publisher _tt_setpoint;
    ros::Publisher _hov_setpoint;

    ros::Subscriber _waypoint_sub;
    ros::Subscriber _state_sub;

    float _thrust;
    float _startZ;
    float _freq;

    double get( const ros::NodeHandle& n, const std::string& name) {
        double value;
        n.getParam(name, value);
        return value;
    }

}; // class LQR

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    std::string worldFrame;
    std::string frame;
    double frequency;

    n.param<std::string>("worldFrame", worldFrame, "/world");
    n.param<std::string>("frame",      frame,      "/rigid_body_1");
    n.param<double>("frequency",       frequency,  300);

    LQR controller(worldFrame, frame, frequency, n);
    controller.spin();

    return 0;
}
