#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

class PositionController:
    def __init__(self):
        #rospy.Subscriber('/mocap_node/Drone/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/vrpn_client_node/DDD/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/control_effort', Float64, self.effort_callback)

        self.pid_error_pub = rospy.Publisher('/state', Float64, queue_size=1)
        self.set_point_pub = rospy.Publisher('/setpoint', Float64, queue_size=1)

        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=1)

        self.reference_position = [0, 0, 1]
        self.P_GAIN = 0.6
        self.z_bias = 0.5

        self.joy_throttle_output = -1

    def pose_callback(self, pose_data):
        self.z_error = self.reference_position[2] - pose_data.pose.position.z
        self.pid_error_pub.publish(self.z_error)
        self.set_point_pub.publish(Float64(data=0.0))

    def effort_callback(self, effort_data):
        self.joy_throttle_output = (self.z_bias*2-1) + effort_data.data
        pub_joy_msg = Joy(axes=[0,0,0,0,0,0,0])
        pub_joy_msg.axes[1] = self.joy_throttle_output

        self.joy_pub.publish(pub_joy_msg)
        rospy.loginfo(pub_joy_msg)

if __name__ == '__main__':
    rospy.init_node('position_controller', anonymous=True)
    position_controller = PositionController()
    rospy.spin()
