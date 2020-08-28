#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mode = "read"
# mode = "write"

if mode is "read":
    f = open("pose.txt", 'r')
else:
    f = open("pose.txt", 'w')

def pose_callback(pose_data):
    data = "{} {} {}\n".format(pose_data.pose.position.x+0.5, pose_data.pose.position.y-0.1, pose_data.pose.position.z)
    f.write(data) 
 
rospy.init_node('position_controller', anonymous=True)

if mode is "write":
    rospy.Subscriber('/vrpn_client_node/Dh_drone/pose', PoseStamped, pose_callback, queue_size=1)
    while not rospy.is_shutdown():
        continue
    f.close()

else:
    position_x = []
    position_y = []
    position_z = []

    while True:
        line = f.readline()
        if not line: break
        data = line.split()
        position_x.append(float(data[0]))
        position_y.append(float(data[1]))
        position_z.append(float(data[2]))
    f.close()

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    reference_x = [0, 1, 1, 0, 0]
    reference_y = [0, 0, 1, 1, 0]
    reference_z = [0.6, 0.6, 0.6, 0.6, 0.6]
    ax.plot(position_x, position_y, position_z)
    ax.plot(reference_x, reference_y, reference_z)
    ax.set_zlim(0.0, 1.0)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title("Trajectory of Drone")
    plt.legend(["Trajectory", "Reference"])
    plt.show()
