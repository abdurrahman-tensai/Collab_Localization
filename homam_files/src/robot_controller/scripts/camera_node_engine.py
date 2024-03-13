#!/usr/bin/env python

from robot_controller.msg import DetectedObject
from models import TRTModule  # isort:skip

import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image

from models.torch_utils import det_postprocess
from models.utils import blob, letterbox

classNames = ['turtlebot', 'rosbot', '3D printer', 'small chair', 'big chair', 'small table', 'big table 1', 'big table 2', 'big table 3', 'person', 'big bin', 'medium bin', 'small bin']

class CameraPublisher:
    def __init__(self):
        # Initialize the node with rospy
        rospy.init_node('camera_publisher')

        # Initializing the engine
        self.device = torch.device("cuda:0")
        self.Engine = TRTModule("yolo8s.engine", self.device)
        self.H, self.W = self.Engine.inp_info[0].shape[-2:]

        # set desired output names order
        self.Engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])

        # Create publisher. We publish to the /camera topic
        self.publisher = rospy.Publisher('/camera', DetectedObject, queue_size=10)     

        # Subscribe to the /camera/image_raw topic
        rospy.Subscriber("/camera/image_raw", Image, self.callback)

    def callback(self, data):
        isThereDetectedLandmarks = False
        # Convert Image message to OpenCV image
        frame = np.fromstring(data.data, np.uint8).reshape(data.height, data.width, -1)

        # Preprocess the frame for YOLO
        frame, ratio, dwdh = letterbox(frame, (self.W, self.H))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        tensor = blob(rgb, return_seg=False)
        dwdh = torch.tensor(dwdh * 2, dtype=torch.float32, device=self.device)
        tensor = torch.tensor(tensor, device=self.device)

        # Inference
        data = self.Engine(tensor)

        bboxes, scores, labels = det_postprocess(data)
        bboxes -= dwdh
        bboxes /= ratio

        for (bbox, score, label) in zip(bboxes, scores, labels):
            isThereDetectedLandmarks = True
            bbox = bbox.round().int().tolist()
            cls_id = int(label)
            cls = classNames[cls_id]
            self.publisher.publish(DetectedObject(bbox[0], bbox[1], bbox[2], bbox[3], cls))
        if not isThereDetectedLandmarks:
            self.publisher.publish(DetectedObject(-1, -1, -1, -1, "NULL"))

    def run(self):
        # Spin until interrupted
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CameraPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
