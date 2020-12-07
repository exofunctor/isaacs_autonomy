import sys
import rospy

import roslib
roslib.load_manifest('isaacs_autonomy')

from stream_position import StreamPosition
from stream_attitude import StreamAttitude
from stream_camera import StreamCamera

from depth_map import DepthMap
from grid import Grid
from segmentation import Segmentation
from path_planner import PathPlanner


# Start the image visualizer node.
def main(args):

    mission_accomplished = False

    # position_topic = "/dji_sdk/gps_position"
    position_topic = "/dji_sdk/rtk_position"
    attitude_topic = "/dji_sdk/attitude"
    disparity_topic = "/dji_sdk/stereo_240p_front_depth_images"
    segmentation_topic = "/dji_sdk/fpv_camera_images"


    stream_position = StreamPosition(position_topic)
    attitude_stream = AttitudeStream(attitude_topic)
    disparity_stream = CameraStream(disparity_topic)
    segmentation_stream = CameraStream(segmentation_topic)

    depth_map = DepthMap(disparity_stream, attitude_stream)
    grid = Grid(args[0])  # TODO: correct syntax?
    segmentation = Segmentation(segmentation_stream)
    path_planner = PathPlanner

    while not mission_accomplished:
        # TODO: On a separate thread, break if segmentation finds target.
        grid.update_grid(depth_map())
        path_planner.explore(grid.grid)

    # The mission has been accomplished. Time to land!
    print("Mission accomplished!")
    # TODO: /dji_sdk/drone_task_control 6


if __name__ == '__main__':
    # args: radius
    main(sys.argv)


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
