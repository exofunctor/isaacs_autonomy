#!/usr/bin/env python
import time
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
                 service_get_auth="/dji_sdk/sdk_control_authority",
                 service_control="/dji_sdk/drone_task_control",
                 topic_position_control="/dji_sdk/flight_control_setpoint_ENUposition_yaw",
                 disparity_focal_length=None,
                 disparity_FOV=np.pi/3):

        print("[INFO]: Initializing Explorer")

        # When the delivery target has been found, this will turn True.
        self.mission_accomplished = False

        # Store the model's parameters.
        self.UAV_diameter = UAV_diameter
        self.topic_position = topic_position
        self.topic_attitude = topic_attitude
        self.topic_disparity = topic_disparity
        self.service_get_auth = service_get_auth
        self.service_control = service_control
        self.topic_position_control = topic_position_control
        self.disparity_focal_length = disparity_focal_length
        self.disparity_FOV = disparity_FOV
        self.height = 1.2
        self.radius = search_radius

        # Start the sensor streamers.
        # Position @ 50 hz
        self.StreamPosition = StreamPosition(self.topic_position, UAV_diameter)
        print("[INFO]: StreamPosition OK")
        # Attitude @ 100 hz
        self.StreamAttitude = StreamAttitude(self.topic_attitude)
        print("[INFO]: StreamAttitude OK")
        # Disparity @ 10 hz
        self.StreamDisparity = StreamCamera(self.topic_disparity, "mono8")
        print("[INFO]: StreamDisparity OK")

        # Start the ROS node corresponding to this package.
        rospy.init_node("isaacs_autonomy", anonymous=True)

        # Start the control services and publisher.
        self.get_auth = rospy.ServiceProxy(self.service_get_auth, SDKControlAuthority)
        self.control = rospy.ServiceProxy(self.service_control, DroneTaskControl)
        self.position_control = rospy.Publisher(self.topic_position_control, Joy, queue_size=10)

        # Get control over the physical drone.
        if (self.get_auth(1)):
            print("[INFO]: Authority OK")
        else:
            raise Exception("[ERROR]: Authority FAIL. Exiting...")

        # Initialize a depth map.
        self.DepthMap = DepthMap()
        print("[INFO]: DepthMap OK")

        # Initialize a geographical search grid.
        self.Grid = Grid(search_radius)
        print("[INFO]: Grid OK")

        # Update the map with the reference frame measurements.
        self.update_map(True)
        print("[INFO]: World Model OK")

    # Call this function to update the map that the UAV uses to navigate.
    # Ideally, it should be called every time that the UAV advances a tile,
    # or when it performs a sharp turn.
    def update_map(self):
        self.DepthMap.update(self.StreamDisparity.image,
                             self.StreamAttitude.pitch_x,
                             self.StreamAttitude.roll_z,
                             self.disparity_focal_length)

        self.Grid.update(self.DepthMap.depth_map,
                         self.disparity_FOV,
                         self.StreamAttitude.yaw_y,
                         self.StreamPosition.x,
                         self.StreamPosition.z)

    # Perform a floodfill search.
    def explore(self):
        print("[INFO]: Beginning search.")
        max_x = self.Grid.grid.shape[1]
        max_z = self.Grid.grid.shape[0]
        threshold = 0.3
        self.traversed = np.zeros((max_x+4, max_z+4))
        x = self.StreamPosition.x
        z = self.StreamPosition.z

        if (self.takeoff()):
            print("Taking off")
        else:
            print("Failed to take off")

        def is_open(x, z):
            if x < 0 or z < 0 or x >= max_x or z >= max_z:
                return False
            if self.traversed[int(x), int(z)] > 0:
                print("x: ", int(x), "z: ", int(z), "is OCCUPIED/TRAVELED")
                return False
            occupied = self.Grid.grid[int(x), int(z)]
            if occupied == 0:
                #rotate and update map
                curr_x = self.StreamPosition.x
                curr_z = self.StreamPosition.z
                occupied = self.Grid.grid[int(x), int(z)]
            print("x: ", x, "z: ", z, "is OPEN. going there next")
            return  occupied > threshold #returns false if occupied = 0 for safety reasons

        def flood_fill(x,z):
            try:
                if self.check_mission_accomplished():
                    return
                self.update_map()
                if is_open(x, z+1):
                    self.move_up(x, z+1)
                    flood_fill(x,z+1)
                    if self.check_mission_accomplished():
                        return
                    self.move_down(x, z)
                #need to come back to where we started from, because we can't jump around the map
                if is_open(x, z-1):
                    self.move_down(x, z-1)
                    flood_fill(x,z-1)
                    if self.check_mission_accomplished():
                        return
                    self.move_up(x, z)
                if is_open(x+1, z):
                    self.move_right(x+1, z)
                    flood_fill(x+1,z)
                    if self.check_mission_accomplished():
                        return
                    self.move_left(x,z)
                if is_open(x-1, z):
                    self.move_left(x-1,z)
                    flood_fill(x-1, z)
                    if self.check_mission_accomplished():
                        return
                    self.move_right(x,z)
                return
            except KeyboardInterrupt:
                print("Exiting...")

        flood_fill(x, z)

    def takeoff(self):
        return self.control(4)

    def land(self):
        return self.control(6)

    def move_up(self, desired_x, desired_z):
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        print("moving forward, from z=", z, " to z=", desired_z)

        #first, rotate so we are facing up(forwards)
        self.rotate(0)

        #go from (x,z) to (x, z+1)
        while(abs(desired_z - z) > 0.2):
            self.set_z(desired_z - z)
            z = self.StreamPosition.z
        self.set_z(0)

        #update traversal matrix
        self.traversed[int(desired_x), int(desired_z)] = 1
        print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")

        #update occupancy grid
        self.update_map()

    def move_left(self, desired_x, desired_z):
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        print("moving left, from x=", x, " to x=", desired_x)

        self.rotate((3/2)*np.pi)
        #TODO: move
        while(abs(x - desired_x) > 0.2):
            self.set_x(x - desired_x)
            x = self.StreamPosition.x
        self.set_x(0)

        self.traversed[int(desired_x), int(desired_z)] = 1
        print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")

        self.update_map()

    def move_right(self,  desired_x, desired_z):
        print("moving right")
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        print("moving right, from x=", x, " to x=", desired_x)
        #rotate
        self.rotate((1/2)*np.pi)
        #move
        while(abs(x - desired_x) > 0.2):
            self.set_x(x - desired_x)
            x = self.StreamPosition.x
        self.set_x(0)
        self.traversed[int(desired_x), int(desired_z)] = 1
        print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")

        self.update_map()

    def move_down(self, desired_x, desired_z):
        x = self.StreamPosition.x
        z = self.StreamPosition.z
        print("moving backwards, from z=", z, " to z=", desired_z)
        #rotate
        self.rotate(np.pi)
        #move
        while(abs(desired_z - z) > 0.2):
            #print("current z: ", z)
            #print("trying to go to: ", desired_z)
            self.set_z(desired_z - z)
            z = self.StreamPosition.z
        self.set_z(0)

        self.traversed[int(desired_x), int(desired_z)] = 1
        print("we have now visited (x,z) = (", int(desired_x), ", ", int(desired_z), ")")

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
            self.set_yaw(desired_yaw + error) #if our yaw is lower, we increase it. if its too high, we decrease it
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
            self.get_auth(0)
        return acc


# Start the exploration.
def main(args):
    Explorer(args[1])
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")


if __name__ == '__main__':
    main(sys.argv)
