/******************************************************************************
    * lqr.hpp -- Class definition that contains LQR gains                     *
    *                                                                         *
    * Carnegie Mellon Univesity                                               *
    *                                                                         *
    * Author:   Papincak, Lawrence                                            *
    *                                                                         *
    * Usage:                                                                  *
    **************************************************************************/

#include "ros/ros.h"

#define NUM_STATES 12

#define X_STATE         0
#define Y_STATE         1
#define Z_STATE         2
#define X_DOT_STATE     3
#define Y_DOT_STATE     4
#define Z_DOT_STATE     5
#define ROLL_STATE      6
#define PITCH_STATE     7
#define YAW_STATE       8
#define ROLL_DOT_STATE  9
#define PITCH_DOT_STATE 10
#define YAW_DOT_STATE   11

#define THRUST_STATE    0
#define TORQUE_X_STATE  1
#define TORQUE_Y_STATE  2
#define TORQUE_Z_STATE  3

class LQR_gains
{
public:
    LQR_gains(
              double x,
              double y,
              double z,
              double roll,
              double pitch,
              double yaw,
              double dx,
              double dy,
              double dz,
              double droll,
              double dpitch,
              double dyaw)
              : _x(x)
              , _y(y)
              , _z(z)
              , _roll(roll)
              , _pitch(pitch)
              , _yaw(yaw)
              , _dx(dx)
              , _dy(dy)
              , _dz(dz)
              , _droll(droll)
              , _dpitch(dpitch)
              , _dyaw(dyaw)
    {
        gains.push_back( _x );
        gains.push_back( _y );
        gains.push_back( _z );
        gains.push_back( _dx );
        gains.push_back( _dy );
        gains.push_back( _dz );
        gains.push_back( _roll );
        gains.push_back( _pitch );
        gains.push_back( _yaw );
        gains.push_back( _droll );
        gains.push_back( _dpitch );
        gains.push_back( _dyaw );
    }

    double _x;
    double _y;
    double _z;
    double _roll;
    double _pitch;
    double _yaw;
    double _dx;
    double _dy;
    double _dz;
    double _droll;
    double _dpitch;
    double _dyaw;

    std::vector<double> gains;

}; // LQR_gains
