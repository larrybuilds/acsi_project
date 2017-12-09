#!/usr/bin/env python
import time
import rospy
from keyboard.msg import Key
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
from std_msgs.msg import Float32

class Controller():
    def __init__(self, use_controller, key_topic):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.Subscriber("/crazyflie/xyKd", Float32, self.xyKdCallback,  queue_size=1)
        rospy.Subscriber("/crazyflie/xyKi", Float32, self.xyKiCallback,  queue_size=1)
        rospy.Subscriber("/crazyflie/xyKp", Float32, self.xyKpCallback,  queue_size=1)

        # rospy.Subscriber("/crazyflie/yKd", Float32, self.yKdCallback,  queue_size=1)
        # rospy.Subscriber("/crazyflie/yKi", Float32, self.yKiCallback,  queue_size=1)
        # rospy.Subscriber("/crazyflie/yKp", Float32, self.yKpCallback,  queue_size=1)

        rospy.Subscriber("/crazyflie/zKd", Float32, self.zKdCallback,  queue_size=1)
        rospy.Subscriber("/crazyflie/zKi", Float32, self.zKiCallback,  queue_size=1)
        rospy.Subscriber("/crazyflie/zKp", Float32, self.zKpCallback,  queue_size=1)

        rospy.set_param("posCtlPid/xKp", 2.0)
        rospy.set_param("posCtlPid/yKp", 2.0)
        self._update_params(["posCtlPid/xKp"])
        self._update_params(["posCtlPid/yKp"])

        rospy.set_param("posCtlPid/xKd", 0)
        rospy.set_param("posCtlPid/yKd", 0)
        self._update_params(["posCtlPid/xKd"])
        self._update_params(["posCtlPid/yKd"])

        # Reset the kalman filter
        rospy.loginfo("resetting kalman estimator")
        rospy.set_param("kalman/resetEstimation", 1)
        self._update_params(["kalman/resetEstimation"])
        time.sleep(0.1)
        rospy.set_param("kalman/resetEstimation", 0)
        self._update_params(["kalman/resetEstimation"])
        rospy.loginfo("kalman estimator reset")
        rospy.set_param("flightmode/posSet", 1)
        self._update_params(["flightmode/posSet"])



        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
            rospy.loginfo("waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo("found land service")
            self._land = rospy.ServiceProxy('land', Empty)

            rospy.loginfo("waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo("found takeoff service")
            self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        else:
            self._land = None
            self._takeoff = None

        # subscribe to the keyboard input at the end to make sure that all required
        # services were found
        self._code = 0
        rospy.Subscriber(key_topic, Key, self._keyChanged)

    def _keyChanged(self, data):
	if self._code != data.code:
		# Space bar means emergency requested
		if data.code == 32:
			self._emergency()
		# l key means land requested
		if data.code == 108:
			self._land()
		# t key means takeoff requested
		if data.code == 116:
			self._takeoff()

        self._code = data.code

    def xyKdCallback(self,msg):
        rospy.set_param("posCtlPid/xKd", msg.data)
        rospy.set_param("posCtlPid/yKd", msg.data)
        self._update_params(["posCtlPid/xKd"])
        self._update_params(["posCtlPid/yKd"])

    def xyKiCallback(self,msg):
        rospy.set_param("posCtlPid/xKi", msg.data)
        rospy.set_param("posCtlPid/yKi", msg.data)
        self._update_params(["posCtlPid/xKi"])
        self._update_params(["posCtlPid/yKi"])

    def xyKpCallback(self,msg):
        rospy.set_param("posCtlPid/xKp", msg.data)
        rospy.set_param("posCtlPid/yKp", msg.data)
        self._update_params(["posCtlPid/xKp"])
        self._update_params(["posCtlPid/yKp"])

    # def yKdCallback(self,msg):
    #     rospy.set_param("posCtlPid/yKd", msg.data)
    #     self._update_params(["posCtlPid/yKd"])
    #
    # def yKiCallback(self,msg):
    #     rospy.set_param("posCtlPid/yKi", msg.data)
    #     self._update_params(["posCtlPid/yKi"])
    #
    # def yKpCallback(self,msg):
    #     rospy.set_param("posCtlPid/yKp", msg.data)
    #     self._update_params(["posCtlPid/yKp"])

    def zKdCallback(self,msg):
        rospy.set_param("posCtlPid/zKd", msg.data)
        self._update_params(["posCtlPid/zKd"])

    def zKiCallback(self,msg):
        rospy.set_param("posCtlPid/zKi", msg.data)
        self._update_params(["posCtlPid/zKi"])

    def zKpCallback(self,msg):
        rospy.set_param("posCtlPid/zKp", msg.data)
        self._update_params(["posCtlPid/zKp"])


if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    key_topic = rospy.get_param("~key_topic", "/keyboard/keydown")
    controller = Controller(use_controller, key_topic)

    rospy.spin()
