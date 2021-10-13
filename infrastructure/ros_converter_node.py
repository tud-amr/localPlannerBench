import rospy
import numpy as np
import time

from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class ActionConverterNode(object):
    def __init__(self, planner, dt, rate_int, n):
        rospy.init_node("ActionConverter", anonymous=True)
        self._planner = planner
        self._interval = planner._setup_params['interval']
        self._rate_int = rate_int / self._interval
        self._rate = rospy.Rate(self._rate_int)
        self._dt = dt * self._interval
        self._n = n
        self._joint_state_sub = rospy.Subscriber("/joint_states_filtered", JointState, self.joint_state_cb)
        self._acc_pub = rospy.Publisher(
            '/joint_acc_des', 
            Float64MultiArray, queue_size=10
        )
        self._q = np.zeros(self._n)
        self._qdot = np.zeros(self._n)
        self._acc_msg = Float64MultiArray()
        self._acc_msg.data = np.zeros(7)
        self._planner.concretize()

    def joint_state_cb(self, data):
        self._q = np.array(data.position[0:7])
        self._qdot = np.array(data.velocity[0:7])

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


if __name__ == "__main__":
    converterNode = ActionConverterNode(None, 0.01, 100, 7)
    try:
        converterNode.run()
    except rospy.ROSInterruptException:
        pass

