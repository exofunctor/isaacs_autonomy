#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('isaacs_autonomy')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        #self.image_pub = rospy.Publisher("topic2", Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/dji_sdk/fpv_camera_images", Image, self.callback)

    def callback (self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print(cv_image.shape)
        except CvBridgeError as error:
            print(error)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        #try:
        #    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        #except CvBridgeError as error:
        #    print(error)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

 
if __name__ == '__main__':
    main(sys.argv)
