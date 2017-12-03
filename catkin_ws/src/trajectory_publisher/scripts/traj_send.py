#!/usr/bin/env python

import time
import rospy
import logging

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from optitrack.msg import RigidBody, RigidBodyArray

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'
firstPos = False
connectedToCrazyflie = False

_x0 = 0
_y0 = 0
_z0 = 0
_x = 0
_y = 0
_z = 0

# Change the sequence according to your setup
#             x    y    z  YAW
sequence = [
    (2.5, 2.5, 1.2, 0),
    (1.5, 2.5, 1.2, 0),
    (2.5, 2.0, 1.2, 0),
    (3.5, 2.5, 1.2, 0),
    (2.5, 3.0, 1.2, 0),
    (2.5, 2.5, 1.2, 0),
    (2.5, 2.5, 0.4, 0),
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

def rigidBodyCallback(data):
    global firstPos
    global connectedToCrazyflie
    global _x0
    global _y0
    global _z0
    global _x
    global _y
    global _z

    if connectedToCrazyflie == True:
        if firstPos == False:
            _x0 = data.bodies[0].pose.position.x
            _y0 = data.bodies[0].pose.position.y
            _z0 = data.bodies[0].pose.position.z
            print('pos0: ({}, {}, {})'.format(_x0,_y0,_z0))
            firstPos = True
        else:
            _x = data.bodies[0].pose.position.x
            _y = data.bodies[0].pose.position.y
            _z = data.bodies[0].pose.position.z
            scf.cf.extpos.send_extpos(_x, _y, _z)

def sendSetpoint(scf):
    global _x0
    global _y0
    global _z0

    #print('Setpoint X({})  Y({})  Z({})'.format(_x0,_y0,_z0))
    scf.cf.commander.send_setpoint(_y0, _x0, 0, int((_z0+0.5) * 1000))
    #scf.cf.commander.send_hover_setpoint(0, 0, 0, _z0+0.5)

if __name__ == '__main__':

    rospy.init_node('traj_send', anonymous=True)
    rospy.Subscriber("/optitrack/rigid_bodies",RigidBodyArray, rigidBodyCallback)

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

        rospy.loginfo("Recieving optitrack data, sending setpoint")
        while not rospy.is_shutdown():
            sendSetpoint(scf)
            rate.sleep()
