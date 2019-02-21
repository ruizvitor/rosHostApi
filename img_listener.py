#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import CompressedImage
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
import time

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(ros_data):
    print("Received an image!")
    # try:
    #     # Convert your ROS Image message to OpenCV2
    #     cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # except CvBridgeError, e:
    #     print(e)
    # else:
    #     # Save your OpenCV2 image as a jpeg
    #     cv2.imwrite('camera_image.jpeg', cv2_img)

    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    seconds = time.time()
    cv2.imwrite('camera_image'+str(seconds)+'.jpeg', image_np)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    # image_topic = "/cameras/left_hand_camera/image"
    image_topic = "chatterImg"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
