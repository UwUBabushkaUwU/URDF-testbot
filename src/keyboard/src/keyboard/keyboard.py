#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
def joint_controller():
    rospy.init_node('joint_controller')

    # Publisher for each joint
    pub_revolute_15 = rospy.Publisher('/joint1/command', Float64, queue_size=10)
    pub_revolute_16 = rospy.Publisher('/joint2/command', Float64, queue_size=10)
    pub_revolute_17 = rospy.Publisher('/joint3/command', Float64, queue_size=10)
    pub_revolute_18 = rospy.Publisher('/joint4/command', Float64, queue_size=10)

    # Keyboard input handling
    while not rospy.is_shutdown():
        key = input("Enter joint number (1-4) and desired position: ")
        joint_num, position = key.split()

        if joint_num == "1":
            pub_revolute_15.publish(float(position))
        elif joint_num == "2":
            pub_revolute_16.publish(float(position))
        elif joint_num == "3":
            pub_revolute_17.publish(float(position))
        elif joint_num == "4":
            pub_revolute_18.publish(float(position))
        else:
            print("Invalid joint number. Please enter 1, 2, 3, or 4.")

if __name__ == '__main__':
    joint_controller()