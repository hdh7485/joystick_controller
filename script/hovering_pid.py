#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import Empty 
import tf
import math

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
        self.roll_axis = JoyAxis('/roll')
        self.pitch_axis = JoyAxis('/pitch')

        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=1)

        self.reference_xyz = [0, 0, 0]
        self.reference_rpy = [0, 0, 0]
        self.reference_q = [0, 0, 0, 1]

        self.current_xyz = [0, 0, 0]
        self.current_rpy = [0, 0, 0]
        self.current_q = [0, 0, 0, 1]

        self.z_bias = 0.5
        self.takeoff_height = 0.5

        self.joy_throttle_output = -1

        self.state = 'ground'

        rospy.Timer(rospy.Duration(0.001), self.timer_callback)
        rospy.Subscriber('/vrpn_client_node/DDD/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mocap_node/Drone/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber('/target_point', PoseStamped, self.target_point_callback)

    def angle_transform(self, x, y, theta):
        transformed_x = x*math.cos(theta) - y*math.sin(theta)
        transformed_y = x*math.sin(theta) + y*math.cos(theta)
        return [transformed_x, transformed_y]

    def takeoff_callback(self, data):
        if self.state is 'ground':
            self.state = 'takeoff'
        else:
            pass

    def target_point_callback(self, pose_data):
        if self.state is not 'ground':
            self.state = 'target_point'
            q = pose_data.pose.orientation
            self.reference_q = (q.x, q.y, q.z, q.w)
            self.reference_rpy = tf.transformations.euler_from_quaternion(self.current_q)
            self.reference_xyz = [pose_data.pose.position.x, pose_data.pose.position.y, pose_data.pose.position.z]
        else:
            pass

    def pose_callback(self, pose_data):
        self.current_xyz = [pose_data.pose.position.x, pose_data.pose.position.y, pose_data.pose.position.z]
        q = pose_data.pose.orientation
        self.current_q = (q.x, q.y, q.z, q.w)
        self.current_rpy = tf.transformations.euler_from_quaternion(self.current_q)

    def timer_callback(self, time):
        if self.state is 'takeoff':
            self.z_error = self.takeoff_height - self.current_xyz[2]
            self.yaw_error = 0 - self.current_rpy[2]
            [self.roll_error, self.pitch_error] = \
                self.angle_transform(self.reference_xyz[1] - self.current_xyz[1], 
                self.reference_xyz[0] - self.current_xyz[0], self.current_rpy[2])

        elif self.state is 'target_point':
            self.z_error = self.reference_xyz[2] - self.current_xyz[2]
            self.yaw_error = self.reference_rpy[2] - self.current_rpy[2]
            [self.roll_error, self.pitch_error] = \
                self.angle_transform(self.reference_xyz[1] - self.current_xyz[1], 
                self.reference_xyz[0] - self.current_xyz[0], self.current_rpy[2])

        self.throttle_axis.pid_publish(self.z_error, 0)
        self.yaw_axis.pid_publish(self.yaw_error, self.reference_rpy[2])
        self.roll_axis.pid_publish(self.roll_error, self.reference_xyz[1])
        self.pitch_axis.pid_publish(self.pitch_error, self.reference_xyz[0])

        self.joy_throttle_value = (self.z_bias*2-1) + self.throttle_axis.control_effort
        self.joy_yaw_value = self.yaw_axis.control_effort
        self.joy_roll_value = self.roll_axis.control_effort
        self.joy_pitch_value = self.pitch_axis.control_effort

        pub_joy_msg = Joy(axes=[0,0,0,0,0,0,0])

        pub_joy_msg.axes[0] = self.joy_yaw_value
        pub_joy_msg.axes[1] = self.joy_throttle_value
        pub_joy_msg.axes[3] = self.joy_roll_value
        pub_joy_msg.axes[4] = self.joy_pitch_value

        self.joy_pub.publish(pub_joy_msg)

if __name__ == '__main__':
    rospy.init_node('position_controller', anonymous=True)
    position_controller = PositionController()
    rospy.spin()
