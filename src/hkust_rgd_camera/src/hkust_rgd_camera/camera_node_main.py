import os
import numpy as np
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

cv_image = None

def callback(msg):
    global cv_image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
REAL = True
def main():
    global cv_image
    rospy.init_node("camera_node")
    image_publisher = rospy.Publisher("/cam0/image_raw", Image, queue_size=10)
    rospy.Subscriber("/stereo_camera/color/image_raw", Image, callback)
    bridge = CvBridge()
    if REAL:
        cap = cv2.VideoCapture(2)
    
    print("--> Camera node started!")
    last_time = time.time()
    while not rospy.is_shutdown():
        if REAL:
            ret, frame = cap.read()
            if not ret:
                print("--> No image received!")
                continue
        else:
            frame = cv_image
            if frame is None:
                print("--> No image received!")
                continue
            
        cv2.imshow("camera", frame)
        # print(frame.shape)
        k = cv2.waitKey(1)
        if k == ord('q'):
            break
        
        dt = time.time() - last_time
        fps = 1 / dt
        # print("--> FPS {}".format(fps))
        last_time = time.time()
        image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        image_publisher.publish(image_message)