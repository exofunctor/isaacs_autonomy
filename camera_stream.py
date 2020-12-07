#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('isaacs_autonomy')
# import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2



# This class receives a ROS image type and converts it
# to a NumPy and OpenCV compatible format.
class CameraStream:

    def __init__(self, topic):
        self.bridge = CvBridge()
        # topic = "/dji_sdk/stereo_240p_front_depth_images"
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

# encoding can be either bgr8 (color) or mono8 (grayscale)
    def callback(self, data, encoding="bgr8"):
        try:
            # TODO: is gray correct?
            cv2_image = self.bridge.imgmsg_to_cv2(data, encoding)

            #print(cv2_image.shape)
            #return cv2_image
        except CvBridgeError as error:
            print(error)

        #cv2.imshow("Image window", cv2_image)
        #cv2.waitKey(3)
