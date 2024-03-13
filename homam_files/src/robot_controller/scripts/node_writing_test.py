#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class TopicPublisher:
    def __init__(self, topic_name):
        self.topic_name = topic_name

        # Initialize the publisher
        self.pub = rospy.Publisher(self.topic_name, Twist, queue_size=10)

    def publish_data(self, data):
        # Create a String message
        msg = Twist()
        msg.data = data

        # Publish the message
        self.pub.publish(msg)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("topic_publisher_node")

    # Create an instance of the TopicPublisher class
    topic_publisher = TopicPublisher("your_topic_name")  # Replace with your topic name

    # Publish some data
    topic_publisher.publish_data("Hello, ROS!")