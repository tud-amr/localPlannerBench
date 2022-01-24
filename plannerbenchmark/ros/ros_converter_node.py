import rospy
import numpy as np
import time

from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
import std_msgs

class ActionConverterNode(object):
    def __init__(self, dt, rate_int, n):
        rospy.init_node("ActionConverter", anonymous=True)
        self._rate = rospy.Rate(rate_int)
        self._dt = dt
        #boxer
        self._n = 3
        self._nu = 2
        self._stateIndices = [0, 1, 2]
        self._qdotIndices = [3, 4]

        #panda
        #self._n = n
        #self._nu = n
        #self._stateIndices = [0, 1, 2, 3, 4, 5, 6]
        #self._qdotIndices = []
        self._joint_state_sub = rospy.Subscriber("/joint_states_filtered", JointState, self.joint_state_cb)
        self._acc_pub = rospy.Publisher(
            '/joint_acc_des', 
            Float64MultiArray, queue_size=100
        )
        self._stop_pub = rospy.Publisher(
            '/motion_stop_request',
            Bool, queue_size=10
        )
        self._x = np.zeros(self._n)
        self._xdot = np.zeros(self._n)
        self._qdot = np.zeros(self._nu)
        self._acc_msg = Float64MultiArray()
        self._acc_msg.data = np.zeros(self._nu)

    def joint_state_cb(self, data):
        self._x = np.array([data.position[i] for i in self._stateIndices])
        self._xdot = np.array([data.velocity[i] for i in self._stateIndices])
        self._qdot = np.array([data.velocity[i] for i in self._qdotIndices])

    def ob(self):
        return {'x': self._x, 'xdot': self._xdot, 'vel': self._qdot}, rospy.get_time()

    def publishAction(self, action):
        for i in range(self._nu):
            self._acc_msg.data[i] = action[i]
        self._acc_pub.publish(self._acc_msg)
        self._rate.sleep()
        return self.ob()

    def stopMotion(self):
        rospy.loginfo("Stopping ros converter")
        stop_msg = std_msgs.msg.Bool(data=False)
        print(rospy.is_shutdown())
        for i in range(10):
            rospy.loginfo("Stoping node")
            self._stop_pub.publish(stop_msg)
            self._rate.sleep()
            #self.sendZeroAction()

    def run(self):
        self._rate.sleep()
        while not rospy.is_shutdown():
            t_before = time.time()
            acc_des = self._planner.computeAction(self._q, self._qdot)
            t_planning = time.time() - t_before
            #print(t_planning)
            for i in range(self._n):
                self._acc_msg.data[i] = acc_des[i]
            self._acc_pub.publish(self._acc_msg)
            self._rate.sleep()
        return []

    def sendZeroAction(self):
        action = np.zeros(self._n)
        for i in range(10):
            self.publishAction(action)


if __name__ == "__main__":
    converterNode = ActionConverterNode(None, 0.01, 100, 3, 2)
    try:
        converterNode.run()
    except rospy.ROSInterruptException:
        pass

