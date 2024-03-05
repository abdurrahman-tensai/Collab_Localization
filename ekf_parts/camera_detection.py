#! /usr/bin/env python

import cv2
import numpy as np

from ultralytics import YOLO
import cv2
import torch
import numpy as np
import math


cap = cv2.VideoCapture((
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

classNames = ['turtlebot', 'rosbot', '3D printer', 'small chair', 'big chair', 'small table', 'big table 1', 'big table 2', 'big table 3', 'person', 'big bin', 'medium bin', 'small bin']


def draw_frames(frame, x1, y1, x2, y2, class_name, confidence):
# put box in cam
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

    # draw the center of the object
    cv2.circle(frame, (int((x1+x2)/2), int((y1+y2)/2)), radius=5, color=(0, 0, 255), thickness=-1)
            
    # draw the center of the image
    cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), radius=5, color=(0, 255, 0), thickness=-1)

            
    # Concatenate class name and confidence
    text = class_name + ' (' + confidence + ')'

    # object details
    org = (x1, y1-5)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.5
    color = (255, 0, 0)
    thickness = 1

    cv2.putText(frame, text, org, font, fontScale, color, thickness)



device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model = YOLO("./yolo8s.pt")
model.to(device)


while True:
    # Capturing video from the camera
    ret, frame = cap.read()

    results = model(frame)
    
    for result in results:
        boxes = result.boxes
        for box in boxes:
            # class name
            cls = int(box.cls[0])
            class_name = classNames[cls]
            # Your existing code for processing the results goes here
            # if class_name != "big bin" or class_name != "medium bin" or class_name != "small bin":
            #     publisher.publish(DetectedObject(-1, -1, -1, -1, "NULL"))
            #     continue

            confidence = math.ceil((box.conf[0]*100))/100
            if confidence > 0.70 and class_name == "small chair":
                
                # box coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                
                draw_frames(frame, int(x1), int(y1), int(x2), int(y2), class_name, str(confidence))
                

                ## Calculating the distance for the small chair
                width  = int(x2) - int(x1)
                height = int(y2) - int(y1)
                area = width * height
                ## The equation is fitted from experimental measurements
                dist = 549*np.exp(-0.0001644*area) + 238.6*np.exp(-0.00001342*area)

                ## Calculating the angle for the small chair
                frame_x = int(frame.shape[1]/2) # the center of the frame; [px]
                obj_x   = int((x1 + x2)/2.0)    # the center of the object; [px]

                i_px = frame_x - obj_x  # the x distance of the object in pixels; [px]
                px2mm = 7.3*(1.0/1000) # conversion factor from pixels to mm
                
                f_mm = 2.85             # focal length of the camera; [mm]

                angle = np.arctan((i_px * px2mm)/f_mm) # calculating the angle; [rad]

                text_msg1 = "THE OBJECT IS:  " + str(class_name) + "  AND ITS CONFIDENCE IS  " + str(confidence)
                text_msg2 = "THE DISTANCE TO THE OBJECT IS:  " + str(dist)
                text_msg3 = "AND ITS ANGLE (radians) IS:  " + str(angle)

                print(text_msg1)
                print(text_msg2)
                print(text_msg3)




    # Showing the video capture
    cv2.imshow('Robot View', frame)


    # Closing & Shutting down when 'q' is pressed
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

cap.release() # Close camera
cv2.destroyAllWindows()








