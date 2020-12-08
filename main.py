#!/usr/bin/env python
import rospy
import sys
import numpy as np
from stream_position import StreamPosition
from stream_attitude import StreamAttitude
from stream_camera import StreamCamera
from depth_map import DepthMap
from grid import Grid

from sensor_msgs.msg import Joy
from dji_sdk.srv import SDKControlAuthority, DroneTaskControl

import time

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
        self.height = 1.2

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
        self.position_control = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUposition_yaw', Joy, queue_size=10)
        self.get_auth = rospy.ServiceProxy("/dji_sdk/sdk_control_authority", SDKControlAuthority)
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

        if (self.set_auth(1)):
            print("got SDK authority")
        else:
            print("failed to obtain authority")

        self.explore()

    # Call this function to update the map that the UAV uses to navigate.
    # Ideally, it should be called every time that the UAV advances a tile,
    # or when it performs a sharp turn.
    def update_map(self, verbose=False):
        disparity = 5*np.ones((240, 240))
        self.DepthMap.update(
                             disparity,
                             #self.StreamDisparity.image,
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


    # TODO, TODO: thread 1, SEMANTIC SEGMENTATION CODE
    # TODO, TODO: thread 2, PATH PLANING CODE and UPDATE GRID CODE

    # The mission has been accomplished. Time to land!
    # print("Mission accomplished!")
    # TODO: /dji_sdk/drone_task_control 6

    def explore(self):
        print("beginning search")
        max_x = self.Grid.grid.shape[1] #2*radius
        max_z = self.Grid.grid.shape[0] #2*radius
        #threshold = 0.3
        threshold = -np.inf
        self.traversed = np.zeros((max_x, max_z))
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        if (self.takeoff()):
            print("Taking off")
        else:
            print("Failed to take off")

        def is_open(x, z):
            if x < 0 or z < 0 or x > max_x or z > max_z:
                return False
            if self.traversed[x, z] > 0:
                return False
            occupied = self.Grid.grid[x, z]
            if occupied == 0:
                #rotate and update map
                curr_x = self.StreamPosition.x
                curr_z = self.StreamPosition.z
                #TODO: calculate angle between curr and deisred
                #TODO: rotate so we face desired
                #TODO #self.update_map() uncomment this after implementing previous 2 comments
                occupied = self.Grid.grid[x, z]
            return  occupied > threshold #returns false if occupied = 0 for safety reasons

        def flood_fill(x,z):
            try:
                if self.check_mission_accomplished():
                    return
                self.update_map()
                if is_open(x, z+1):
                    self.move_up()
                    flood_fill(x,z+1)
                    if self.check_mission_accomplished():
                        return
                    self.move_down()
                #need to come back to where we started from, because we can't jump around the map
                if is_open(x, z-1):
                    self.move_down()
                    flood_fill(x,z-1)
                    if self.check_mission_accomplished():
                        return
                    self.move_up()
                if is_open(x+1, z):
                    self.move_right()
                    flood_fill(x+1,z)
                    if self.check_mission_accomplished():
                        return
                    self.move_left()
                if is_open(x-1, z):
                    self.move_left()
                    flood_fill(x-1, z)
                    if self.check_mission_accomplished():
                        return
                    self.move_right()
                return
            except KeyboardInterrupt:
                print("Exiting...")

        flood_fill(x, z)

    def set_auth(self, status):
        return self.get_auth(status)

    def takeoff(self):
        return self.control(4)

    def land(self):
        return self.control(6)

    def move_up(self):
        #need to go from (x,z) to (x,z+1)
        print("moving forward")
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        desired_x = x
        desired_z = z + 1

        #first, rotate so we are facing up(forwards)
        self.rotate(np.pi)

        #go from (x,z) to (x, z+1)
        while(abs(desired_z - z) > 0.2):
            print("current z: ", z)
            print("trying to go to: ", desired_z)
            self.set_z(desired_z - z)
            z = self.StreamPosition.z
        self.set_z(0)

        #update traversal matrix
        self.traversed[desired_x, desired_z] = 1

        #update occupancy grid
        self.update_map()

    def move_left(self):
        #need to go from (x,z) to (x-1, z)
        print("moving left")
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        desired_x = x - 1
        desired_z = z

        self.rotate((3/2)*np.pi)
        #TODO: move
        while(abs(desired_x - x) > 0.2):
            print("current x: ", x)
            print("trying to go to: ", desired_x)
            self.set_x(desired_x - x)
            x = self.StreamPosition.x
        self.set_x(0)

        self.traversed[desired_x, desired_z] = 1
        self.update_map()

    def move_right(self):
        print("moving right")
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        desired_x = x + 1
        desired_z = z
        #rotate
        self.rotate((1/2)*np.pi)
        #move
        while(abs(desired_x - x) > 0.2):
            print("current x: ", x)
            print("trying to go to: ", desired_x)
            self.set_x(desired_x - x)
            x = self.StreamPosition.x
        self.set_x(0)
        self.traversed[desired_x, desired_z] = 1
        self.update_map()

    def move_down(self):
        print("moving backwards")
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        desired_x = x
        desired_z = z - 1
        #rotate
        self.rotate(0)
        #move
        while(abs(desired_z - z) > 0.2):
            print("current z: ", z)
            print("trying to go to: ", desired_z)
            self.set_z(desired_z - z)
            z = self.StreamPosition.z
        self.set_z(0)

        self.traversed[desired_x, desired_z] = 1
        self.update_map()

    def set_yaw(self, change): #might need to multiply the change value
        msg = Joy()
        msg.axes = [0, 0, self.height, change] #double check these values
        self.position_control.publish(msg)
        time.sleep(1)
        return

    def rotate(self, desired_yaw):
        return
        print("rotating to ", desired_yaw, "radians")
        error = (np.pi)/16
        yaw = self.StreamAttitude.yaw_y
        while (yaw < desired_yaw - error or yaw > desired_yaw + error):
            print("current yaw is ", yaw)
            print("setting yaw to ", desired_yaw)
            self.set_yaw(desired_yaw) #if our yaw is lower, we increase it. if its too high, we decrease it
            yaw = self.StreamAttitude.yaw_y
        self.set_yaw(0) #stop rotating
        return

    def set_z(self, change):
        yaw = self.StreamAttitude.yaw_y
        msg = Joy()
        msg.axes = [0, change, self.height, 0] #double check these values
        self.position_control.publish(msg)
        time.sleep(1)
        return

    def set_x(self, change):
        yaw = self.StreamAttitude.yaw_y
        msg = Joy()
        msg.axes = [change, 0, self.height, 0]
        self.position_control.publish(msg)
        time.sleep(1)
        return

    def check_mission_accomplished(self):
        acc = self.mission_accomplished
        if acc:
            print("Mission accomplished. Landing and removing authority.")
            self.land()
            self.set_auth(0)
        return acc


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
