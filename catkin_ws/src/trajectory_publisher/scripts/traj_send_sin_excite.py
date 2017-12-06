#!/usr/bin/env python

import time
import rospy
import logging
import math

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from geometry_msgs.msg import Point
from optitrack.msg import RigidBody, RigidBodyArray

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'
firstPos = False
connectedToCrazyflie = False

freq = 8.087026648

_x0 = 0
_y0 = 0
_z0 = 0
_x = 0
_y = 0
_z = 0
_xD = 0
_yD = 0
_zD = 0
_ogZ = 0

_bx = 0
_by = 0
_bz = 0

# Change the sequence according to your setup
#             x    y    z    dt
sequence = [
	(0.000000, 0.000000, 0.000000, 0.144459),
	(0.000000, 0.004650, -0.010025, 0.144459),
	(0.000000, 0.025069, -0.030749, 0.144459),
	(0.000000, 0.062209, -0.049978, 0.144459),
	(0.000000, 0.096074, -0.059729, 0.144459),
	(0.000000, 0.103121, -0.061071, 0.144459),
	(0.000000, 0.076354, -0.059798, 0.144459),
	(0.000000, 0.035854, -0.048804, 0.144459),
	(0.000000, 0.020183, -0.022561, 0.144459),
	(0.000000, 0.044830, -0.007123, 0.144459),
	(0.000000, 0.079631, -0.026755, 0.144459),
	(0.000000, 0.084678, -0.057235, 0.144459),
	(0.000000, 0.057219, -0.074535, 0.144459),
	(0.000000, 0.026365, -0.078649, 0.144459),
	(0.000000, 0.009221, -0.072818, 0.144459),
	(0.000000, 0.001852, -0.061888, 0.144459),
	(0.000000, -0.000585, -0.048028, 0.144459),
	(0.000000, -0.000457, -0.030702, 0.144459),
	(0.000000, -0.000064, -0.010501, 0.144459),
	(0.000000, 0.000000, 0.000000, 0.144459),
]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def wait_for_position_estimator():
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(self._scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    self.wait_for_position_estimator()


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def extPoseCallback(data):
    global firstPos
    global connectedToCrazyflie
    global _x0
    global _y0
    global _z0
    global _xD
    global _yD
    global _zD
    global _x
    global _y
    global _z
    global _ogZ

    if connectedToCrazyflie == True:
        if firstPos == False:
            _x0 = data.x
            _y0 = data.y
            _z0 = 1.2
            _ogZ = data.z

            _xD = _x0
            _yD = _y0
            _zD = _z0

            firstPos = True
        else:
            _x = data.x
            _y = data.y
            _z = data.z
            scf.cf.extpos.send_extpos(_x, _y, _z)

def ballPoseCallback(data):
    global _bx
    global _by
    global _bz

    if connectedToCrazyflie == True:
        _bx = data.x
        _by = data.y
        _bz = data.z

def rigidBodyCallback(data):
    global firstPos
    global connectedToCrazyflie
    global _x0
    global _y0
    global _z0
    global _xD
    global _yD
    global _zD
    global _x
    global _y
    global _z
    global _ogZ

    if connectedToCrazyflie == True:
        if firstPos == False:
            _x0 = data.bodies[0].pose.position.x
            _y0 = data.bodies[0].pose.position.y
            _z0 = 1.2
            _ogZ = data.bodies[0].pose.position.z
            print('GZ {}'.format(_ogZ))
            _xD = _x0
            _yD = _y0
            _zD = _z0

            firstPos = True
        else:
            _x = data.bodies[0].pose.position.x
            _y = data.bodies[0].pose.position.y
            _z = data.bodies[0].pose.position.z
            scf.cf.extpos.send_extpos(_x, _y, _z)

def setPointCallback(data):
    global _xD
    global _yD
    global _zD

    _xD = data.x
    _yD = data.y
    _zD = data.z

def sendHover(scf):
    global _x0
    global _y0
    global _z0

    #print('Setpoint X({})  Y({})  Z({})'.format(_x0,_y0,_z0))
    scf.cf.commander.send_setpoint(_y0, _x0, 0, int(_z0 * 1000))
    #scf.cf.commander.send_hover_setpoint(0, 0, 0, _z0+0.5)

def sendSetpoint(scf,setpoint):
    global _x0
    global _y0
    global _z0
    scf.cf.commander.send_setpoint(setpoint[1]+_y0, setpoint[0]+_x0, 0, int((setpoint[2]+_z0) * 1000))

if __name__ == '__main__':

    rospy.init_node('traj_send', anonymous=True)
    rospy.Subscriber("/crazyflie/external_position", Point, extPoseCallback)
    rospy.Subscriber("/crazyflie/waypoint", Point, setPointCallback)
    rospy.Subscriber("/crazyflie/ball_position", Point, ballPoseCallback)

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri) as scf:
        connectedToCrazyflie = True
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        cf.param.set_value('flightmode.posSet', '1')

        firstPos = False
        # Wait for first position to come in
        rate = rospy.Rate(20)
        while not firstPos:
            rate.sleep()

        for x in range(50):
            if x == 0:
                x = 25

            sendSetpoint(scf,(0,0,-_ogZ*((50-x)/50.0)))
            time.sleep(0.1)

        for x in range(50):
            sendHover(scf)
            time.sleep(0.1)

        rospy.loginfo("Sending trajectory...")
        t = 0.0
        for x in range(1000):
            sendSetpoint(scf,(0,math.sin(freq*t),0))
            t = t + 0.01
            time.sleep(0.01)

        # for waypoint in sequence:
        #     sendSetpoint(scf,waypoint)
        #     time.sleep(waypoint[3])

        # for x in range(150):
        #     sendSetpoint(scf,(0,math.sin(freq*t - 1.570795),0))
        #     t = t + 0.01
        #     time.sleep(0.01)
        #
        # for x in range(150):
        #     sendSetpoint(scf,(0,math.sin(freq*t),0))
        #     t = t + 0.01
        #     time.sleep(0.01)
        #
        # for x in range(150):
        #     sendSetpoint(scf,(0,math.sin(freq*t - 1.570795),0))
        #     t = t + 0.01
        #     time.sleep(0.01)
        #
        # for x in range(150):
        #     sendSetpoint(scf,(0,math.sin(freq*t),0))
        #     t = t + 0.01
        #     time.sleep(0.01)

        for x in range(50):
            sendHover(scf)
            time.sleep(0.1)

        # rospy.loginfo("Recieving optitrack data, sending setpoint")
        # while not rospy.is_shutdown():
        #     sendSetpoint(scf,(0,0,0))
        #     rate.sleep()

        for x in range(50):
            sendSetpoint(scf,(0,0,-_ogZ*(x/50.0)))
            time.sleep(0.1)

        scf.cf.commander.send_stop_setpoint()
