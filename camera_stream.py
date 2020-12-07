#!/usr/bin/env python
from __future__ import print_function
# import roslib
# import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import cv2

# roslib.load_manifest('isaacs_autonomy')


# This class receives a ROS image type and converts it
# to a NumPy and OpenCV compatible format.
class CameraStream:

    def __init__(self, topic):
        self.bridge = CvBridge()
        # topic = "/dji_sdk/stereo_240p_front_depth_images"
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, data):
        try:
            # TODO: is gray correct?
            cv2_image = self.bridge.imgmsg_to_cv2(data, "gray")
            # print(cv2_image.shape)
            return cv2_image
        except CvBridgeError as error:
            print(error)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(1)
