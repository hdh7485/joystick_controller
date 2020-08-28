#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

class PositionController:
    def __init__(self):
        rospy.Subscriber('/vc/Drone/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mocap_node/Drone/pose', PoseStamped, self.pose_callback)
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=1)

        self.reference_position = [0, 0, 1]
        self.P_GAIN = 0.6
        self.z_bias = 0.6

        self.joy_throttle_output = 0

    def pose_callback(self, pose_data):

        z_error = self.reference_position[3] - pose_data.pose.position.z
        self.joy_throttle_output = (self.z_bias + z_error * self.P_GAIN) * 3 - 1
        pub_joy_msg = Joy(axes=[1,0,0,0,0,0,0])

        pub_joy_msg.axes[2] = self.joy_throttle_output
        self.joy_pub.publish(pub_joy_msg)

        rospy.loginfo(pub_joy_msg)

if __name__ == '__main__':
    rospy.init_node('position_controller', anonymous=True)
    position_controller = PositionController()
    rospy.spin()