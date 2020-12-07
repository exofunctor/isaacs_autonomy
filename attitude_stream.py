#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('isaacs_autonomy')
import rospy
from geometry_msgs.msg import QuaternionStamped
from numpy import array, arctan2, arcsin, pi
from camera_stream import CameraStream

# TODO: Description
# 100hz
class AttitudeStream:
    def __init__(self, topic):
        self.attitude_sub = rospy.Subscriber(topic, QuaternionStamped, self.callback)
        self.euler_angles = array([0, 0, 0])
        self.pitch_x = 0
        self.yaw_y = 0
        self.roll_z = 0
        self.init_euler_angles = array([0, 0, 0])
        self.init = False

    def callback(self, data):
        attitude = (
                    data.quaternion.x,
                    data.quaternion.y,
                    data.quaternion.z,
                    data.quaternion.w
                   )
        euler_angles = self.Q2Euler(attitude)
        # TODO: Description.
        if not self.init:
            self.init_euler_angles = euler_angles
            self.init = True
        self.euler_angles = euler_angles - self.init_euler_angles
        self.pitch_x = self.euler_angles[0]
        self.yaw_y = self.euler_angles[1]
        self.roll_z = self.euler_angles[2]
        print(self.euler_angles * 180 / pi)
        #return self.euler_angles

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
    def Q2Euler(self, Q):
        x = Q[0]
        y = Q[1]
        z = Q[2]
        w = Q[3]
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
    attitude_stream = AttitudeStream("/dji_sdk/attitude")
    camera_stream = CameraStream("/dji_sdk/stereo_240p_front_depth_images") 
    print(attitude_stream.euler_angles)
    print(camera_stream)
    rospy.init_node("isaacs_autonomy", anonymous=True)
    try:
        #print(self.euler_angles)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...") 


if __name__ == '__main__':
    main()
