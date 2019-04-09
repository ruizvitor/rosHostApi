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
# OpenCV2 for saving an image
import cv2
import numpy as np
import time

delta = 0

def img_callback(ros_data):
    global delta
    print(int(round(time.time() * 1000)) - delta)
    delta = int(round(time.time() * 1000))

    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

    cv2.imshow("Image window", image_np)
    cv2.waitKey(1)


def main():

    rospy.init_node('image_listener')
    rospy.Subscriber("camera/compressed", CompressedImage,
                     img_callback,  queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down...')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
