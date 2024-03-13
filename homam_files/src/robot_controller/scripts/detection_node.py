#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import torch
from models import TRTModule  # isort:skip

from models.torch_utils import det_postprocess
from models.utils import blob, letterbox

classNames = ['turtlebot', 'rosbot', '3D printer', 'small chair', 'big chair', 'small table', 'big table 1', 'big table 2', 'big table 3', 'person', 'big bin', 'medium bin', 'small bin']

# Initialize the engine
device = torch.device("cuda:0")
Engine = TRTModule("yolo8s.engine", device)
H, W = Engine.inp_info[0].shape[-2:]

# set desired output names order
Engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])

def draw_frames(frame, x1, y1, x2, y2, class_name, confidence):
# put box in cam
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

    # draw the center of the object
    cv2.circle(frame, (int((x1+x2)/2), int((y1+y2)/2)), radius=5, color=(0, 0, 255), thickness=-1)
            
    # draw the center of the image
    cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), radius=5, color=(0, 255, 0), thickness=-1)

    # object details
    org = [x1, y1-5]
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.5
    color = (255, 0, 0)
    thickness = 1

    cv2.putText(frame, f'{class_name} ({confidence:.3f})', org, font, fontScale, color, thickness)

def image_callback(img_data):
    # Convert the ROS Image message to a numpy array
    frame = np.fromstring(img_data.data, np.uint8).reshape(img_data.height, img_data.width, -1)

    # Preprocess the frame for YOLO
    frame, ratio, dwdh = letterbox(frame, (416, 416))
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    tensor = blob(rgb, return_seg=False)
    dwdh = torch.tensor(dwdh * 2, dtype=torch.float32, device=device)
    tensor = torch.tensor(tensor, device=device)

    # Inference
    data = Engine(tensor)

    bboxes, scores, labels = det_postprocess(data)
    bboxes -= dwdh
    bboxes /= ratio

    for (bbox, score, label) in zip(bboxes, scores, labels):
        bbox = bbox.round().int().tolist()
        cls_id = int(label)
        cls = classNames[cls_id]

        x1, y1, x2, y2 = bbox
        # Draw the bounding boxes on the image
        draw_frames(frame, int(x1), int(y1), int(x2), int(y2), cls, score)

    # Display the image
    cv2.imshow('Robot View', frame)
    cv2.waitKey(1)

# if __name__ == '__main__':
#     # Initialize the node with rospy
#     rospy.init_node('detection_node')

#     # Create a subscriber to the /camera/image_raw topic
#     rospy.Subscriber("/camera/image_raw", Image, image_callback)

#     # Spin until ctrl + c
#     rospy.spin()
