#!/usr/bin/env python
import rospy
import sys
import numpy as np
from stream_position import StreamPosition
from stream_attitude import StreamAttitude
from stream_camera import StreamCamera
from depth_map import DepthMap
from grid import Grid

import math
from sensor_msgs.msg import Joy
from dji_sdk.msg import SDKControlAuthority

#from robotics_final_project.segmentation import Segmentation
#from robotics_final_project.path_planner import PathPlanner
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

        print("[INFO]: Initializing Explorer")

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

        # Start the sensor streamers. TODO TODO: synchronize at 10 hz
        # TODO: ? hz
        self.StreamPosition = StreamPosition(self.topic_position, UAV_diameter)
        print("[INFO]: StreamPosition OK")
        # 100 hz
        self.StreamAttitude = StreamAttitude(self.topic_attitude)
        print("[INFO]: StreamAttitude OK")
        # 10 hz
        self.StreamDisparity = StreamCamera(self.topic_disparity, "mono8")
        print("[INFO]: StreamDisparity OK")
	#print(self.StreamDisparity.image)
        # TODO: ? hz
        self.StreamSegmentation = StreamCamera(self.topic_segmentation, "bgr8")
        print("[INFO]: StreamSegmentation OK")

        # Start the ROS node corresponding to this package.
        rospy.init_node("isaacs_autonomy", anonymous=True)

        # setup publishers and services
        self.position_control = rospy.Publisher('flight_control_setpoint_ENUposition_yaw', Joy, queue_size=10)
        self.get_auth = rospy.Publisher("dji_sdk/sdk_control_authority", SDKControlAuthority)
        self.control = rospy.ServiceProxy("/dji_sdk/drone_task_control", DroneTaskControl)


        # Initialize a depth map.
        self.DepthMap = DepthMap()
        print("[INFO]: DepthMap OK")

        # Initialize a geographical search grid.
        self.Grid = Grid(search_radius)
        print("[INFO]: Grid OK")


        # Update the map with the initial measurements.
        self.update_map(True)
        print("[INFO]: World Model OK")

        self.set_auth(1)
        self.explore()

    # Call this function to update the map that the UAV uses to navigate.
    # Ideally, it should be called every time that the UAV advances a tile,
    # or when it performs a sharp turn.
    def update_map(self, verbose=False):
        try:
            self.DepthMap.update(
                             self.StreamDisparity.image,
                             self.StreamAttitude.pitch_x,
                             self.StreamAttitude.roll_z,
                             self.disparity_focal_length,
                             verbose
                             )

            self.Grid.update(
                         self.DepthMap.depth_map,
                         self.disparity_FOV,
                         self.StreamAttitude.yaw_y,
                         self.StreamPosition.x,
                         self.StreamPosition.z,
                         verbose
                         )
            self.update_map(True)
        except KeyboardInterrupt:
            print("Shutting down...")
            exit


    # TODO, TODO: thread 1, SEMANTIC SEGMENTATION CODE
    # TODO, TODO: thread 2, PATH PLANING CODE and UPDATE GRID CODE

    # The mission has been accomplished. Time to land!
    # print("Mission accomplished!")
    # TODO: /dji_sdk/drone_task_control 6

    def explore(self):
        max_x = self.Grid.grid.shape[1] #2*radius
        max_z = self.Grid.grid.shape[0] #2*radius
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
    #print(args)
    #args = args[1:]
    print(args[1])
    Explorer(args[1])
    #rospy.init_node("isaacs_autonomy", anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")


if __name__ == '__main__':
    main(sys.argv)
