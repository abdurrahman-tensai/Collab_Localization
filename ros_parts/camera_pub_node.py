#! /usr/bin/env python


###############################################################################
#   This file is a basic topic publisher. The main functionality is to 
#   publish the camera data to be seen by other nodes. The output of the file
#   is as follow:
#       + 2 topics (/distance  &  /bearing) the data is an array for the 
#           objects/classes to be detected where any i-th position is for 
#           a known object class.
#       Distance is in cm, bearning angle is in radians
#
#   Written by      : Abdurrahman
#   Last Modified   : 13 / 03 / 2024
#
###############################################################################


import rospy
from std_msgs.msg import Float64MultiArray

class CameraPublisher:
    def __init__(self, distance_topic, bearing_topic):
        self.distance_topic = distance_topic
        self.bearing_topic = bearing_topic

        # Initialize the publishers
        self.distance_pub = rospy.Publisher(self.distance_topic, Float64MultiArray, queue_size=10)
        self.bearing_pub = rospy.Publisher(self.bearing_topic, Float64MultiArray, queue_size=10)
    
    def detect_objects(self):
        # This is where you put your object detection code
        # For now, we'll just return some dummy data
        distances = [10.0, 20.0, 30.0]
        bearings = [45.0, 90.0, 135.0]
        return distances, bearings
    
    def publish_data(self):
        # Detect the objects
        distances, bearings = self.detect_objects()

        # Create Float64MultiArray messages
        distance_msg = Float64MultiArray()
        distance_msg.data = distances
        bearing_msg = Float64MultiArray()
        bearing_msg.data = bearings

        # Publish the messages
        self.distance_pub.publish(distance_msg)
        self.bearing_pub.publish(bearing_msg)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("camera_publisher_node")

    # Create an instance of the CameraPublisher class
    topic_publisher = CameraPublisher("/distance", "/bearing")  # distance and bearing angle topics

    # Publish data in a loop
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        topic_publisher.publish_data()
        rate.sleep()