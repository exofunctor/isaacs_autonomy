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

import math
from sensor_msgs.msg import Joy
from dji_sdk.msg import SDKControlAuthority

# import roslib
# roslib.load_manifest('isaacs_autonomy')


# Matrice 210 Specifications:
# - Diameter: 0.887 meters
# - Forward Horizontal FOV: 60 degrees
# - Forward Vertical FOV: 54 degrees
# - Forward Sensing Range: 0.7-30 meters
class Explorer:

    def __init__(self,
                 search_radius,
                 UAV_diameter=0.887,
                 topic_position="/dji_sdk/gps_position",
                 topic_attitude="/dji_sdk/attitude",
                 topic_disparity="/dji_sdk/stereo_240p_front_depth_images",
                 disparity_focal_length=None,
                 disparity_FOV=np.pi/3,
                 topic_segmentation="/dji_sdk/fpv_camera_images",
                 ):

        # When the delivery target has been found, this will turn True.
        self.mission_accomplished = False

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

        # setup publishers and services
        self.position_control = rospy.Publisher('flight_control_setpoint_ENUposition_yaw', Joy)
        self.get_auth = rospy.Publisher("dji_sdk/sdk_control_authority", SDKControlAuthority)
        self.control = rospy.ServiceProxy("/dji_sdk/drone_task_control", DroneTaskControl)


        # Initialize a depth map.
        self.DepthMap = DepthMap()

        # Initialize a geographical search grid.
        self.Grid = Grid(search_radius)


        # Update the map with the initial measurements.
        self.update_map()

        self.set_auth(1)
        self.explore()

    # Call this function to update the map that the UAV uses to navigate.
    # Ideally, it should be called every time that the UAV advances a tile,
    # or when it performs a sharp turn.
    def update_map(self):
        self.DepthMap.update(
                             self.disparity.image,
                             self.StreamAttitude.pitch_x,
                             self.StreamAttitude.roll_z,
                             self.disparity_focal_length
                             )

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

    def explore(self):
        max_x = self.Grid.grid.shape(1) #2*radius
        max_z = self.Grid.grid.shape(0) #2*radius
        threshold = 0.3
        self.traversed = np.zeros((max_x, max_z))
        position_control = rospy.Publisher('flight_control_setpoint_ENUposition_yaw', Joy)
        get_auth = rospy.Publisher("dji_sdk/sdk_control_authority", SDKControlAuthority)



    def set_auth(self, status):
        self.get_auth(status)

    def takeoff(self):
        self.control(4)

    def land(self):
        self.control(6)


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
