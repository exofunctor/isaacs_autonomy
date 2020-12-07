#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import cv2


# This class receives a ROS image type and converts it
# to a NumPy and OpenCV compatible format.
class CameraStream:

    def __init__(self, topic, encoding="bgr8"):

        """
        - topic:    The topic to subscribe to and output as a stream.
                    On the M210, this is either:
                        - /dji_sdk/stereo_240p_front_depth_images
                        - /dji_sdk/fpv_camera_images

        - encoding: The encoding of the image. Choose "bgr8" for uint8 BGR,
                    or "mono8" for uint8 grayscale.
        """

        self.topic = topic

        # The current image and its parameters.
        self.image = None
        self.encoding = encoding
        self.height = 0
        self.width = 0

        # The bridge that converts ROS Image topics to OpenCV/NumPy arrays.
        self.bridge = CvBridge()

        # The subscriber, listening to an Image topic.
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, self.encoding)
            self.image = image
            self.height, self.width = image.shape[:2]
            # print(self.height, self.width)
            # cv2.imshow(self.topic, image)
            # cv2.waitKey(3)
        except CvBridgeError as error:
            print(error)
