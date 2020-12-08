import rospy
import sys
import numpy as np
from stream_position import StreamPosition
from stream_attitude import StreamAttitude
from stream_camera import StreamCamera
from depth_map import DepthMap
from grid import Grid
from segmentation import Segmentation
from path_planner import PathPlanner
# import roslib
# roslib.load_manifest('isaacs_autonomy')


class Explorer:

    def __init__(self,
                 search_radius,
                 UAV_diameter=0.8,
                 topic_position="/dji_sdk/gps_position",
                 topic_attitude="/dji_sdk/attitude",
                 topic_disparity="/dji_sdk/stereo_240p_front_depth_images",
                 disparity_focal_length=None,
                 disparity_FOV=np.pi/4,
                 topic_segmentation="/dji_sdk/fpv_camera_images",
                 ):

        # Store the model's parameters.
        self.UAV_diameter = UAV_diameter
        self.topic_position = topic_position
        self.topic_attitude = topic_attitude
        self.topic_disparity = topic_disparity
        self.disparity_focal_length = disparity_focal_length
        self.disparity_FOV = disparity_FOV
        self.topic_segmentation = topic_segmentation

        # Start the sensor streamers.
        self.StreamPosition = StreamPosition(topic_position, UAV_diameter)
        self.StreamAttitude = StreamAttitude(topic_attitude)
        self.StreamDisparity = StreamCamera(topic_disparity, "mono8")
        self.StreamSegmentation = StreamCamera(topic_segmentation, "bgr8")

        # Initialize a depth map.
        self.DepthMap = DepthMap()
        self.DepthMap.update(
                             self.disparity.image,
                             self.StreamAttitude.pitch_x,
                             self.StreamAttitude.roll_z,
                             self.disparity_focal_length
                             )

        # Initialize a geographical search grid.
        self.Grid = Grid(search_radius)
        self.Grid.update(
                         self.DepthMap.depth_map,
                         self.disparity_FOV,
                         self.StreamAttitude.yaw_y,
                         self.StreamPosition.x,
                         self.StreamPosition.z
                         )

        # TODO, TODO: thread 1, SEMANTIC SEGMENTATION CODE
        # TODO, TODO: thread 2, PATH PLANING CODE and UPDATE GRID CODE

        # The mission has been accomplished. Time to land!
        # print("Mission accomplished!")
        # TODO: /dji_sdk/drone_task_control 6


# Start the Exploration.
def main(args):

    Explorer(**args)
    rospy.init_node("isaacs_autonomy", anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")


if __name__ == '__main__':
    main(sys.argv)
