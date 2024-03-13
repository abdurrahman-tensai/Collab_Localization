#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraNode:
    def __init__(self):
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.cap = cv2.VideoCapture((
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                0,
                416,
                416,
                30,
                2,
                416,
                416,
            )
        ), cv2.CAP_GSTREAMER)

    def main(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                # Convert frame to Image message without cv_bridge
                msg_frame = Image()
                msg_frame.height = frame.shape[0]
                msg_frame.width = frame.shape[1]
                msg_frame.encoding = "bgr8"
                msg_frame.is_bigendian = False
                msg_frame.step = len(frame[0])
                msg_frame.data = np.array(frame).tostring()

                self.image_pub.publish(msg_frame) # Publish Image message
                # cv2.imshow('Robot View', frame)
            else:
                rospy.logerr("Unable to capture video frame")
        self.cap.release() # Close camera

if __name__ == '__main__':
    rospy.init_node('camera_node')
    camera_node = CameraNode()
    camera_node.main()
