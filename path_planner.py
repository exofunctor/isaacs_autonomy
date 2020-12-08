
import rospy
import numpy as np
import dji_control
import math
from sensor_msgs.msg import Joy # <-- might need fixing


max_x = 20 #2*radius
max_z = 20 #2*radius
threshold = 0.3
grid = np.zeros((max_x,max_z)) #occupancy grid
traversed = np.zeros((max_x, max_z)) #have we already been there?

pub = rospy.Publisher('flight_control_setpoint_ENUposition_yaw', Joy)

def flood_fill(x,z):
    if is_open(x, z+1):
        move_up()
        flood_fill(x,z+1)
        move_backward()
        #need to come back to where we started from, because we can't jump around the map
    if is_open(x, z-1):
        move_down()
        flood_fill(x,z-1)
        move_up()
    if is_open(x+1, z):
        move_right()
        flood_fill(x+1,z)
        move_left()
    if is_open(x-1, z):
        move_left()
        flood_fill(x,y-1)
        move_right()

def is_open(x,z):
    if x < 0 or z < 0 or x > max_x or z > max_z:
        return False
    if traversed[x, z] > 0:
        return False
    return grid.grid[x, z] < threshold

def move_up():
    x = position_stream.get_x
    z = position_stream.get_z
    desired_x = x
    desired_z = z + 1
    #need to go from (x,z) to (x,z+1)

    #first, rotate so we are facing up(forwards)
    yaw = attitude_stream.yaw #should this be yaw_x or yaw_z or just yaw
    while (yaw < (15/16)*math.pi  or yaw > (17/16)*math.pi):
        set_yaw(math.pi - yaw) #if our yaw is lower, we increase it. if its too high, we decrease it
        yaw = attitude_stream.yaw
    set_yaw(0) #stop rotating

    #go from (x,z) to (x, z+1)
    while(abs(desired_z - z) > 0.2):
        set_z(desired_z - z)
        z = position_stream.get_z
    set_z(0)

    traversed[x,z] = 1

def move_left():
    #need to go from (x,z) to (x-1, z)
    #rotate
    #move

    traversed[x,z] = 1

def move_right():
    #rotate
    #move

    traversed[x,z] = 1

def move_down():
    #rotate
    #move

    traversed[x,z] = 1


def set_yaw(change): #might need to multiply the change value
    msg = Joy()
    msg.axes = [0, 0, 0, change] #double check these values
    pub.publish(msg)

def set_z(change):
    msg = Joy()
    msg.axes = [0, change, 0, 0] #double check these values
    pub.publish(msg)

def set_x(change):
    msg = Joy()
    msg.axes = [change, 0, 0, 0]
    pub.publish(msg)
