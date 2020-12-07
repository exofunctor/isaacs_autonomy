#!/usr/bin/env python
import rospy
from geometry_msgs.msg import QuaternionStamped
import numpy as np


# Stream the attitude of the UAV.
# RATE: 100 hz
class AttitudeStream:
    def __init__(self, topic):
        """
        - topic: The topic to subscribe to and output as a stream.
                 On the M210, this is: /dji_sdk/attitude
        """

        self.topic = topic

        # Whether this stream has been initialized yet.
        self.init = False

        # The initial readings of the subscriber, in Euler angles,
        # from where the UAV started.
        self.init_euler_angles = np.array([0, 0, 0])

        # The current readings of the subscriber, in Euler angles.
        # These are normalized according to the initial attitude;
        # that is, their are **local** angles, **not global**.
        # They measure the difference from the initial attitude.
        self.euler_angles = np.array([0, 0, 0])
        self.pitch_x = 0
        self.yaw_y = 0
        self.roll_z = 0

        # The subscriber, listening to a QuaternionStamped topic.
        self.attitude_sub = rospy.Subscriber(topic, QuaternionStamped,
                                             self.callback)

    def callback(self, data):
        # Decompose the message into a known data type (tuple).
        attitude = (
                    data.quaternion.x,
                    data.quaternion.y,
                    data.quaternion.z,
                    data.quaternion.w
                   )

        # Convert the Quaternion readings into Euler angles.
        euler_angles = self.Q2Euler(attitude)

        # If this is the first reading, save it to normnalize
        # any subsequent readings.
        if not self.init:
            self.init_euler_angles = euler_angles
            self.init = True

        # Normalize this sample.
        self.euler_angles = euler_angles - self.init_euler_angles
        self.pitch_x = self.euler_angles[0]
        self.yaw_y = self.euler_angles[1]
        self.roll_z = self.euler_angles[2]
        # print(self.euler_angles * 180 / np.pi)

    # Convert a quaternion `Q` into euler angles.
    def Q2Euler(self, Q):

        """
        - Q: A quaternion, stored as an indexable structure,
             such as a set or NumPy array.

        """

        # Pitch is the rotation around the x-axis in **radians**.
        # Yaw is the rotation around the y-axis in **radians**.
        # Roll is the rotation around the z-axis in **radians**.

        #
        #  y (yaw)
        #  ^
        #  |   ^ z (roll - where the UAV's front camera is facing)
        #  |  /
        #  | /
        #  |/
        # -|----------> x (pitch)
        #

        # Positive angle values correspond to **counterclockwise** directions.

        x = Q[0]
        y = Q[1]
        z = Q[2]
        w = Q[3]

        t = 2. * (w*y - z*x)
        t = 1. if t > 1. else t
        t = -1. if t < -1. else t
        pitch_x = np.arcsin(t)

        t0 = 2. * (w*z + x*y)
        t1 = 1. - 2. * (y**2 + z**2)
        yaw_y = np.arctan2(t0, t1)

        t0 = 2. * (w*x + y*z)
        t1 = 1. - 2. * (x**2 + y**2)
        roll_z = -np.arctan2(t0, t1)

        euler_angles = np.array([pitch_x, yaw_y, roll_z])
        return euler_angles
