import rospy
import numpy as np
import time

from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
import std_msgs

from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningEnv.sphereObstacle import SphereObstacle


class ActionConverterNode(object):
    def __init__(self, dt, rate_int, robotType):
        rospy.init_node("ActionConverter", anonymous=True)
        self._rate = rospy.Rate(rate_int)
        self._dt = dt

        if robotType == 'panda':
            self._n = 7
            self._nu = 7
            self._actionIndices = [2, 3, 4, 5, 6, 7, 8]
            self._stateIndices = [5, 6, 7, 8, 9, 10, 11]
            self._qdotIndices = []
            self._root_frame = "panda_link0"
        elif robotType == 'boxer':
            self._n = 3
            self._nu = 2
            self._actionIndices = [0, 1]
            self._stateIndices = [0, 1, 2]
            self._qdotIndices = [3, 4]
            self._root_frame = "map"
        elif robotType == 'albert':
            self._n = 10
            self._nu = 9
            self._actionIndices = [0, 1, 2, 3, 4, 5, 6, 7, 8]
            self._stateIndices = [0, 1, 2, 5, 6, 7, 8, 9, 10, 11]
            self._qdotIndices = [3, 4]
            self._root_frame = "map"
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
        # fixed message date size for the albert robot, some remain zeros 
        self._acc_msg.data = np.zeros(9)
        self.initMarker()

    def initMarker(self):
        self._goal_pub = rospy.Publisher(
            '/bench/goal', 
            Marker, queue_size=10
        )
        self._goal_marker = Marker()
        self._goal_marker.header.frame_id = self._root_frame
        self._goal_marker.type = Marker.SPHERE
        self._goal_marker.action = Marker.ADD
        self._goal_marker.color.a = 1.0
        self._goal_marker.color.r = 0.0
        self._goal_marker.color.g = 1.0
        self._goal_marker.color.b = 0.0
        self._obst_pub = rospy.Publisher(
            '/bench/obst', 
            MarkerArray, queue_size=10
        )
        self._obst_markers = MarkerArray()
        self._obst_counter = 0
        self.initObstMarkers(2)

    def initObstMarkers(self, nbObst):
        for i in range(nbObst):
            m = self.initObstMarker(i)
            self._obst_markers.markers.append(m)

    def initObstMarker(self, i):
        marker = Marker()
        marker.header.frame_id = self._root_frame
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    def joint_state_cb(self, data):
        self._x = np.array([data.position[i] for i in self._stateIndices])
        self._xdot = np.array([data.velocity[i] for i in self._stateIndices])
        self._qdot = np.array([data.velocity[i] for i in self._qdotIndices])

    def ob(self):
        return {'x': self._x, 'xdot': self._xdot, 'vel': self._qdot}, rospy.get_time()

    def setGoal(self, goal, t=0):
        self._goal_marker.pose.position.x = goal.position(t=t)[0]
        self._goal_marker.pose.position.y = goal.position(t=t)[1]
        self._goal_marker.pose.position.z = goal.position(t=t)[2]
        self._goal_marker.scale.x = goal.epsilon()
        self._goal_marker.scale.y = goal.epsilon()
        self._goal_marker.scale.z = goal.epsilon()


    def setObstacle(self, obst: SphereObstacle, i, t=0):
        self._obst_markers.markers[i].pose.position.x = obst.position(t=t)[0]
        self._obst_markers.markers[i].pose.position.y = obst.position(t=t)[1]
        self._obst_markers.markers[i].pose.position.z = obst.position(t=t)[2]
        self._obst_markers.markers[i].scale.x = obst.radius()
        self._obst_markers.markers[i].scale.y = obst.radius()
        self._obst_markers.markers[i].scale.z = obst.radius()

    def publishAction(self, action):
        for i in range(self._nu):
            self._acc_msg.data[self._actionIndices[i]] = action[i]
        self._acc_pub.publish(self._acc_msg)
        self._goal_pub.publish(self._goal_marker)
        self._obst_pub.publish(self._obst_markers)
        self._rate.sleep()
        return self.ob()

    def stopMotion(self):
        rospy.loginfo("Stopping ros converter")
        stop_msg = std_msgs.msg.Bool(data=False)
        for i in range(10):
            rospy.loginfo("Stoping node")
            self._stop_pub.publish(stop_msg)
            self._rate.sleep()
