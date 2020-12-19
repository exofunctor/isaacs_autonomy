#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
import numpy as np
import pymap3d as pm


# Stream the attitude of the UAV.
# RATE: 50 hz
class StreamPosition:
    def __init__(self, topic, UAV_diameter=0.887):
        """
        - topic:    The topic to subscribe to and output as a stream.
                    On the M210, this is either:
                        - /dji_sdk/gps_position
                        - /dji_sdk/rtk_position

        - diameter: The diameter of the UAV, in meters.
                    The diameter of the M210 is 0.887 meters.
        """

        self.topic = topic
        self.UAV_diameter = UAV_diameter

        # Whether this stream has been initialized yet.
        self.init = False

        # The initial readings of the subscriber, in WGS84 coordinates,
        # from where the UAV started.
        self.init_lat = 0
        self.init_lon = 0
        self.init_alt = 0

        # The current readings of the subscriber, in ENU xzy,
        # These are normalized according to the initial position;
        # that is, their are **local** measurements, **not global**.
        # They measure the difference from the initial position.
        self.position = np.array([0, 0, 0])
        self.x = 0
        self.y = 0
        self.z = 0

        # The subscriber, listening to a QuaternionStamped topic.
        self.position_sub = rospy.Subscriber(topic, NavSatFix,
                                             self.callback)

    def callback(self, data):

        # Decompose the message into known data types.
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude

        # If this is the first reading, save it to normnalize
        # any subsequent readings.
        if not self.init:
            self.init_lat = lat
            self.init_lon = lon
            self.init_alt = alt
            self.init = True

        x, z, y = pm.geodetic2enu(lat, lon, alt,
                                  self.init_lat, self.init_lon, self.init_alt)

        # In NumPy, indexing starts from the top-left. We therefore invert E
        # to get a WNU reading, which more closely approximates NumPy indexing.
        x = -x

        # Normalize the measurements according to the UAV's diameter,
        # to have it such that every one unit of the xyz data represents
        # a cube of dimension UAV_diameter**3
        self.position = np.array([x, y, z]) / self.UAV_diameter
        self.x = self.position[0]
        self.y = self.position[1]
        self.z = self.position[2]
