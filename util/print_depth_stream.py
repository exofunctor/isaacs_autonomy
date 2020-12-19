#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from stream_attitude import StreamAttitude
from depth_map import DepthMap


# This class receives a ROS image type and converts it
# to a NumPy and OpenCV compatible format.
class DepthVisualizer:

    def __init__(self):
        self.bridge = CvBridge()
        topic = "/dji_sdk/stereo_240p_front_depth_images"
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.StreamAttitude = StreamAttitude("/dji_sdk/attitude")
        self.DepthMap = DepthMap()

    def callback(self, data):
        try:
            disparity = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as error:
            print(error)

        disparity_gauss = cv2.GaussianBlur(disparity/255., (5, 5), 10)
        disparity_med = cv2.medianBlur(disparity, 5)/255.
        disparity = disparity/255.

        depth_map = self.DepthMap.warp3D(disparity,
                                         -self.StreamAttitude.pitch_x,
                                         -self.StreamAttitude.roll_z,
                                         None)

        depth_map1 = self.DepthMap.warp3D(disparity_gauss,
                                          -self.StreamAttitude.pitch_x,
                                          -self.StreamAttitude.roll_z,
                                          None)

        depth_map2 = self.DepthMap.warp3D(disparity_med,
                                          -self.StreamAttitude.pitch_x,
                                          -self.StreamAttitude.roll_z,
                                          None)

        top = np.hstack([depth_map, depth_map1, depth_map2])
        bottom = np.hstack([disparity, disparity_gauss, disparity_med])
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
