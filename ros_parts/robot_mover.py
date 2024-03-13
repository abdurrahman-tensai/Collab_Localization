#! /usr/bin/env python


###############################################################################
#   This file is a basic robot mover. The main functionality is to 
#   control the movement of the robot and move it in a specific path 
#
#   Written by      : Abdurrahman
#   Last Modified   : 13 / 03 / 2024
#
###############################################################################

import rospy
from geometry_msgs.msg import Twist

class RobotMover:
    def __init__(self, cmd_vel_topic):
        self.cmd_vel_topic = cmd_vel_topic

        # Initialize the publisher
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

    def move_in_circle(self):
        # Create a Twist message
        twist = Twist()

        # Linear velocity (forward) in m/s
        twist.linear.x = 0.1

        # Angular velocity (turn) in rad/s
        twist.angular.z = 0.05  # relation between radius, v, and w: v = r * w

        # Publish the message
        self.pub.publish(twist)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("robot_mover_node")

    # Create an instance of the RobotMover class
    circle_mover = RobotMover("/cmd_vel")  # Publishing command to the velocity topic

    # Move in a circle
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        circle_mover.move_in_circle()
        rate.sleep()