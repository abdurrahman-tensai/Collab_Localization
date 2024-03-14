#! /usr/bin/env python

###############################################################################
#   This file is a basic topic reader. The main functionality is to 
#   read data from specific topics for debugging purposes. It also saves the 
#   data to be later plotted/processed to make sure everything is fine.
#   This will be lated the basic structure for the node that will perform 
#   the EKF as input data will be needed.
#
#   Written by      : Abdurrahman
#   Last Modified   : 13 / 03 / 2024
#
###############################################################################

import rospy
from nav_msgs.msg import Odometry  # message typr to get /odom data
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf


class TopicReader:
    def __init__(self, topic_name):
        ## Initializing the variables to be used for storing data
        self.pose_data     = [] # robot pose data: x, y, theta
        self.distance_data = [] # camera distance data to objects
        self.bearing_data  = [] # camera bearing angle data to objects

        self.topic_name = topic_name

        # Initialize the subscriber
        self.sub = rospy.Subscriber(self.topic_name, Odometry, self.odom_callback) 
        self.dist_sub = rospy.Subscriber("/distance", Float64MultiArray, self.dist_callback)
        self.angle_sub = rospy.Subscriber("/bearing", Float64MultiArray, self.angle_callback)

    def odom_callback(self, msg):
        # # Append the data to the list
        # self.data.append(msg.data)
        # Extract the pose and twist from the message
        pose = msg.pose.pose
        twist = msg.twist.twist

        # Update the state of the robot
        self.odom_x = pose.position.x
        self.odom_y = pose.position.y
        quater = (pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w)
        self.odom_theta = tf.transformations.euler_from_quaternion(quater) # Convert the orientation to Euler (3x1)
        self.odom_theta = self.odom_theta[2]

        ## Printing the data on the terminal for debugging
        print(" =========================================================== ")
        print("POSE X:  " + str(self.odom_x) + "     POSE Y:  " + str(self.odom_y) + "      THETA:  " + str(self.odom_theta))

        ## Append the data to the list to later be saved
        self.pose_data.append([self.odom_x, self.odom_y, self.odom_theta])
        
    def dist_callback(self, msg):
        distances = msg.data

        print("DISTANCE:   " + str(distances))

        ## Append the data to the list to later be saved
        self.distance_data.append(distances)

    def angle_callback(self, msg):
        bearings = msg.data

        print("BEARNING ANGLE:   " + str(bearings))

        ## Append the data to the list to later be saved
        self.bearing_data.append(bearings)

    def save_data(self):
        ## Convert the lists to numpy arrays
        np_pose = np.array(self.pose_data)
        np_distance = np.array(self.distance_data)
        np_bearing = np.array(self.bearing_data)

        ## Save the numpy arrays to CSV files
        np.savetxt('pose_data.csv', np_pose, delimiter=',')
        np.savetxt('distance_data.csv', np_distance, delimiter=',')
        np.savetxt('bearing_data.csv', np_bearing, delimiter=',')

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("topic_reader_node")

    # Create an instance of the TopicReader class
    topics_reader  = TopicReader("/odom")  # Replace with your topic name


    # Spin until Ctrl+C is pressed
    rospy.spin()

    # After spinning is done, save the data
    topics_reader.save_data()