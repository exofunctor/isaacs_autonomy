
import rospy
import numpy as np
import dji_control
import math
from sensor_msgs.msg import Joy
from dji_sdk.msg import SDKControlAuthority


max_x = self.Grid.grid.shape(1) #2*radius
max_z = self.Grid.grid.shape(0) #2*radius
threshold = 0.3
self.traversed = np.zeros((max_x, max_z)) #have we already been there?

self.position_control = rospy.Publisher('flight_control_setpoint_ENUposition_yaw', Joy)
self.get_auth = rospy.Publisher("dji_sdk/sdk_control_authority", SDKControlAuthority)
self.control = rospy.ServiceProxy("/dji_sdk/drone_task_control", DroneTaskControl)

set_auth(1)

def flood_fill(x,z):
    if self.mission_accomplished:
        land()
        set_auth(0)
        return
    self.update_map()
    if is_open(x, z+1):
        move_up()
        flood_fill(x,z+1)
        if self.mission_accomplished:
            land()
            set_auth(0)
            return
        move_backward()
        #need to come back to where we started from, because we can't jump around the map
    if is_open(x, z-1):
        move_down()
        flood_fill(x,z-1)
        if self.mission_accomplished:
            land()
            set_auth(0)
            return
        move_up()
    if is_open(x+1, z):
        move_right()
        flood_fill(x+1,z)
        if self.mission_accomplished:
            land()
            set_auth(0)
            return
        move_left()
    if is_open(x-1, z):
        move_left()
        flood_fill(x,y-1)
        if self.mission_accomplished:
            land()
            set_auth(0)
            return
        move_right()
    return

def is_open(x,z):
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
        self.update_map()
        occupied = self.Grid.grid[x, z]
    return  occupied > threshold #returns false if occupied = 0 for safety reasons

def move_up():
    #need to go from (x,z) to (x,z+1)
    x = self.StreamPosition.x
    z = self.StreamPosition.z
    desired_x = x
    desired_z = z + 1

    #first, rotate so we are facing up(forwards)
    rotate(math.pi)
    
    #go from (x,z) to (x, z+1)
    while(abs(desired_z - z) > 0.2):
        set_z(desired_z - z)
        z = self.StreamPosition.z
    set_z(0)

    #update traversal matrix
    self.traversed[desired_x, desired_z] = 1

    #update occupancy grid
    self.update_map()

def move_left():
    #need to go from (x,z) to (x-1, z)
    #TODO: rotate
    #TODO: move

    self.traversed[desired_x, desired_z] = 1
    self.update_map()

def move_right():
    #TODO: rotate
    #TODO: move
    self.traversed[desired_x, desired_z] = 1
    self.update_map()

def move_down():
    #TODO: rotate
    #TODO: move
    self.traversed[desired_x, desired_z] = 1
    self.update_map()

def rotate(desired_yaw):
    error = (math.pi)/16
    yaw = self.StreamAttitude.yaw_y
    while (yaw < desired_yaw - error or yaw > desired_yaw + error:
        set_yaw(desired_yaw - yaw) #if our yaw is lower, we increase it. if its too high, we decrease it
        yaw = self.StreamAttitude.yaw_y
    set_yaw(0) #stop rotating
    return

def set_yaw(change): #might need to multiply the change value
    msg = Joy()
    msg.axes = [0, 0, 0, change] #double check these values
    position_control.publish(msg)

def set_z(change):
    msg = Joy()
    msg.axes = [0, change, 0, 0] #double check these values
    position_control.publish(msg)

def set_x(change):
    msg = Joy()
    msg.axes = [change, 0, 0, 0]
    position_control.publish(msg)

def set_auth(status):
    get_auth(status)
