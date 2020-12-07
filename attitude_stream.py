#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion
from numpy import array, arctan2, arcsin


# TODO: Description
class AttitudeStream:
    def __init__(self, topic):
        self.attitude_sub = rospy.Subscriber(topic, Quaternion, self.callback)
        self.initial_euler_angles = # TODO

    def callback(self, data):
        euler_angles = self.Q2Euler(data.x, data.y, data.z, data.w)
        # TODO: Description.
        local_euler_angles = euler_angles - self.initial_euler_angles
        print(local_euler_angles)
        return local_euler_angles

    # Convert a quaternion into euler angles.
    # Pitch is the rotation around the x-axis in radians.
    # Yaw is the rotation around the y-axis in radians.
    # Roll is the rotation around the z-axis in radians.
    #
    #  y (yaw)
    #  ^   z (pitch - where the UAV's front camera is facing)
    #  |  /
    #  | /
    #  |/
    # ―|――――――――> x (pitch)
    #
    # TODO: determine rl correspondances
    def Q2Euler(self, x, y, z, w):
        """
        - x, y, z, w: the values of the quaternion.

        """
        t0 = 2. * (w*x + y*z)
        t1 = 1. - 2. * (x**2 + y**2)
        roll_z = arctan2(t0, t1)

        t2 = 2. * (w*y - z*x)
        t2 = 1. if t2 > 1. else t2
        t2 = -1. if t2 < -1. else t2
        pitch_x = arcsin(t2)

        t3 = 2. * (w*z + x*y)
        t4 = 1. - 2. * (y**2 + z**2)
        yaw_y = arctan2(t3, t4)

        euler_angles = array([pitch_x, yaw_y, roll_z])
        return euler_angles
