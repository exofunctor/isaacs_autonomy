#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('isaacs_autonomy')
import rospy
from geometry_msgs.msg import QuaternionStamped
from numpy import array, arctan2, arcsin, pi


# TODO: Description
# 100hz
class AttitudeStream:
    def __init__(self, topic):
        self.attitude_sub = rospy.Subscriber(topic, QuaternionStamped, self.callback)
        self.euler_angles = 0
        self.init_euler_angles = 0
        self.init = False

    def callback(self, data):
        euler_angles = self.Q2Euler(data)
        # TODO: Description.
        if not self.init:
            self.init_euler_angles = euler_angles
            self.init = True
        self.euler_angles = euler_angles - self.init_euler_angles
        print(self.euler_angles * 180 / pi)
        return self.euler_angles

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
    # -|--------> x (pitch)
    #
    # TODO: determine rl correspondances
    def Q2Euler(self, quaternion_stamped):
        x = quaternion_stamped.quaternion.x
        y = quaternion_stamped.quaternion.y
        z = quaternion_stamped.quaternion.z
        w = quaternion_stamped.quaternion.w
        """
        - x, y, z, w: the values of the quaternion.

        """
        t0 = 2. * (w*x + y*z)
        t1 = 1. - 2. * (x**2 + y**2)
        roll_z = -arctan2(t0, t1)

        t2 = 2. * (w*y - z*x)
        t2 = 1. if t2 > 1. else t2
        t2 = -1. if t2 < -1. else t2
        pitch_x = arcsin(t2)

        t3 = 2. * (w*z + x*y)
        t4 = 1. - 2. * (y**2 + z**2)
        yaw_y = arctan2(t3, t4)

        euler_angles = array([pitch_x, yaw_y, roll_z])
        return euler_angles


def main():
    AttitudeStream("/dji_sdk/attitude")
    rospy.init_node("attitude_stream", anonymous=True)
    try:
        #print(self.euler_angles)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...") 


if __name__ == '__main__':
    main()
