import numpy as np
import cv2

# from geometry_msgs.msg import Point


# NOTE: yaw might be in degrees - check!
class Explorer():

    def __init__(self, origin, grid_radius):
        # self.position = self.update_position()
        # self.initial_position = self.position
        # self.position = self.position - self.initial_position
        self.rtk_init_position = self.initialize_position()
        self.rtk_init_orientation = self.initialize_orientation()
        length = self.grid_radius2length(grid_radius)
        self.grid = self.create_grid(length)
        self.grid_radius = grid_radius
        self.mission_accomplished = False

    # def update_position(self):
        # Subscribe to DJI SDK
        # and convert such as to be relative to the grid,
        # and/or initial_position
        # return 0

    def initialize_position(self):
        # TODO: Subscribe to DJI SDK
        # and convert such as to be relative to the grid.
        rtk_init_position = 0
        return rtk_init_position

    def grid_radius2length(self, grid_radius):
        length = grid_radius  # TODO: adjust for relative size
        length = np.int(length)
        return

    # Create a square grid of dimensions `length * length`.
    def create_grid(self, length):
        # Initialize grid entries to 0, to indicate that
        # they have not been explored yet.
        grid = np.zeros((length, length), dtype=np.int16)
        return grid


# TODO: description and parameters.
def update_grid(grid, depth, yaw, x, y):

    # Calculate how far the obstacle is from the robot based on the yaw
    # and the given depth. Then, convert these distances to the absolute
    # x and y coordinates of the obstacle in the world, by adding to it
    # the robot's position.

    #
    #         . <- obstacle
    #    /   /
    # depth /
    #   /  /
    #     /) <- yaw
    #    . ――――
    #    ^
    #    |
    #   x, y (robot position)
    #

    dx = depth * np.cos(yaw)
    dy = depth * np.sin(yaw)
    obstacle_x = x + dx
    obstacle_y = y + dy

    # Convert the position of the obstacle to grid coordinates.
    grid_y = grid.shape[0] - 1
    obstacle_x = int(np.ceil(obstacle_x))
    obstacle_y = grid_y - int(np.ceil(obstacle_y))

    # NumPy and OpenCV begin indexing from the top-left corner, which means
    # that the robot's y must be adjusted to match grid coordinates.
    x = int(np.floor(x))
    y = int(np.floor(grid_y - y))

    # In NumPy, the first index selects the rows (y-axis),
    # while the second index selects the columns (x-axis).
    grid[obstacle_y][obstacle_x] = grid[obstacle_y][obstacle_x] + 1

    # Create a box, large enough to fit the depth size, in any orientation,
    # and then draw the depth line on it. The resulting array will
    # be used to update all grid pixels up to where the depth ends -
    # that is, where the sensor has identified that an obstacle is present.
    box = np.zeros((-(obstacle_y - y), obstacle_x - x))

    # Draw the depth line on top of the box. This is equivalent
    # to setting to "1" all the pixels traversed by the depth line.
    # Unlike NumPy, in most OpenCV functions, the first dindex indicates
    # the Nth column (x-axis), and the second index the Nth row (y-axis).
    box_y = box.shape[0] - 1
    cv2.line(box, (0, box_y), (int(dx), box_y - int(dy)),
             color=1, thickness=1)

    # Negate the box to show that the space indicated by the line that
    # was drawn is empty (unoccupied).
    box = -box.astype(grid.dtype)

    # Finally, add the box to its corresponding area in the grid,
    # to indicate that all pixels traversed by the depth line
    # have been identified as empty.
    x1 = x
    x2 = obstacle_x
    y1 = obstacle_y + 1
    y2 = y + 1
    # TODO: Only works for square grids. Perhaps expand to rectangles.
    grid[y1:y2, x1:x2] = grid[y1:y2, x1:x2] + box

    return grid


##
##
# Subscribed topics
# /dji_sdk/rtk_position
# ARGUMENTS
# radius
