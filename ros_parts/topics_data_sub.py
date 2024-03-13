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
# import pandas as pd
import tf


class TopicReader:
    def __init__(self, topic_name):
        self.data = []
        self.topic_name = topic_name

        # Initialize the subscriber
        self.sub = rospy.Subscriber(self.topic_name, Odometry, self.odom_callback) 

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

        print(" ======================================= ")
        print("POSE X:  " + str(self.odom_x) + "     POSE Y:  " + str(self.odom_y) + "      THETA:  " + str(self.odom_theta))


        self.distance = [10.0, 20.0, 30.0]
        self.bearing  = [0, -0.25, 0.25]

        print("DISTANCE:   " + str(self.distance) + "      BEARNING ANGLE:   " + str(self.bearing))

    ## This would be lates used to save the data 
    # def save_data(self):
    #     # Convert the list to a DataFrame
    #     df = pd.DataFrame(self.data, columns=['Data'])

    #     # Save the DataFrame to a CSV file
    #     df.to_csv('data_test.csv', index=False)

    #     # Uncomment the following lines to save data as a rosbag
    #     # bag = rosbag.Bag('data.bag', 'w')
    #     # try:
    #     #     for d in self.data:
    #     #         bag.write(self.topic_name, d)
    #     # finally:
    #     #     bag.close()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("topic_reader_node")

    # Create an instance of the TopicReader class
    topic_reader = TopicReader("/odom")  # Replace with your topic name

    # Spin until Ctrl+C is pressed
    rospy.spin()

    # After spinning is done, save the data
    # topic_reader.save_data()