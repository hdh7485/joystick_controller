#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import tf

class JoyAxis:
    def __init__(self, ns=''):
        self.control_effort = 0
        self.state = 0
        self.setpoint = 0

        rospy.Subscriber(ns+'/control_effort', Float64, self.effort_callback, queue_size=1)
        self.pid_error_pub = rospy.Publisher(ns+'/state', Float64, queue_size=1)
        self.set_point_pub = rospy.Publisher(ns+'/setpoint', Float64, queue_size=1)

    def pid_publish(self, state, setpoint):
        self.state = state
        self.setpoint = setpoint
        self.pid_error_pub.publish(state)
        self.set_point_pub.publish(setpoint)

    def effort_callback(self, effort_data):
        self.control_effort = effort_data.data

    #@state.setter
    #def state(self, state):
    #    self.__state = state

    #@setpoint.setter
    #def setpoint(self, setpoint):
    #    self.__setpoint = setpoint

    #def conrtol_effort(self):
    #    return self.__control_effort

class PositionController:
    def __init__(self):
        #rospy.Subscriber('/mocap_node/Drone/pose', PoseStamped, self.pose_callback)
        
        self.throttle_axis = JoyAxis('/throttle')
        self.yaw_axis = JoyAxis('/yaw')

        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=1)

        self.reference_xyz = [0, 0, 1]
        self.reference_rpy = [0, 0, 0]

        self.current_xyz = [0, 0, 0]
        self.current_rqy = [0, 0, 0]
        self.current_q = [0, 0, 0, 0]

        self.z_bias = 0.5

        self.joy_throttle_output = -1

        rospy.Timer(rospy.Duration(0.001), self.timer_callback)
        rospy.Subscriber('/vrpn_client_node/DDD/pose', PoseStamped, self.pose_callback)
        #rospy.Subscriber('/control_effort', Float64, self.effort_callback)

        rospy.Subscriber('/mocap_node/Drone/pose', PoseStamped, self.pose_callback)

    def pose_callback(self, pose_data):
        self.current_xyz = [pose_data.pose.position.x, pose_data.pose.position.y, pose_data.pose.position.z]

        self.z_error = self.reference_xyz[2] - self.current_xyz[2]

        q = pose_data.pose.orientation
        self.current_q = (q.x, q.y, q.z, q.w)
        self.current_rpy = tf.transformations.euler_from_quaternion(self.current_q)
        self.yaw_error = self.reference_rpy[2] - self.current_rpy[2]

        self.throttle_axis.pid_publish(self.z_error, 0)
        self.yaw_axis.pid_publish(self.yaw_error, self.reference_rpy[2])

    def timer_callback(self, time):
        self.joy_throttle_value = (self.z_bias*2-1) + self.throttle_axis.control_effort
        self.joy_yaw_value = self.yaw_axis.control_effort

        pub_joy_msg = Joy(axes=[0,0,0,0,0,0,0])

        pub_joy_msg.axes[0] = self.joy_yaw_value
        pub_joy_msg.axes[1] = self.joy_throttle_value

        self.joy_pub.publish(pub_joy_msg)

if __name__ == '__main__':
    rospy.init_node('position_controller', anonymous=True)
    position_controller = PositionController()
    rospy.spin()
