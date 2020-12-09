#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from stream_attitude import StreamAttitude


# This class receives a ROS image type and converts it
# to a NumPy and OpenCV compatible format.
class DepthVisualizer:

    def __init__(self):
        self.bridge = CvBridge()
        topic = "/dji_sdk/stereo_240p_front_depth_images"
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.StreamAttitude = StreamAttitude("/dji_sdk/attitude")

    def callback(self, data):
        try:
            disparity = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as error:
            print(error)

        disparity_gauss1 = cv2.GaussianBlur(disparity, 25, 10)
        disparity_gauss2 = cv2.GaussianBlur(disparity, 5, 30)

        depth_map = self.warp3D(disparity,
                                self.StreamAttitude.pitch_x,
                                self.StreamAttitude.roll_z,
                                None)

        depth_map1 = self.warp3D(disparity_gauss1,
                                 self.StreamAttitude.pitch_x,
                                 self.StreamAttitude.roll_z,
                                 None)

        depth_map2 = self.warp3D(disparity_gauss2,
                                 self.StreamAttitude.pitch_x,
                                 self.StreamAttitude.roll_z,
                                 None)

        top = np.hstack([disparity, disparity_gauss1, disparity_gauss2])
        bottom = np.hstack([depth_map, depth_map1, depth_map2])
        out = np.vstack([top, bottom])

        cv2.imshow("Disparity", out)
        cv2.waitKey(3)


# Start the image visualizer node.
def main():
    DepthVisualizer()
    rospy.init_node('depth_visualizer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
