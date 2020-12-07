#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

roslib.load_manifest('isaacs_autonomy')


# This class receives a ROS image type and converts it
# to a NumPy and OpenCV compatible format.
class image_converter:

    def __init__(self, topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print(cv_image.shape)
        except CvBridgeError as error:
            print(error)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


# Start the image visualizer node.
def main(topic):
    image_converter(topic)
    rospy.init_node('image_visualizer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
