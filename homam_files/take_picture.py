import cv2
import os

# Define the gstreamer pipeline
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=416,
    capture_height=416,
    display_width=416,
    display_height=416,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# Initialize the image counter
img_counter = 502

# Create the images directory if it doesn't exist
if not os.path.exists('images_test'):
    os.makedirs('images_test')

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

while(True):
    ret, frame = cap.read()

    cv2.imshow('frame', frame)

    k = cv2.waitKey(1) & 0xFF

    # If 'c' is pressed, capture an image
    if k == ord('c'):
        img_name = "images/image_test.jpg"
        cv2.imwrite(img_name, frame)
        print("image written!")
        img_counter += 1

    # If 'q' is pressed, break from the loop
    elif k == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
