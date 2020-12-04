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
# NOTE: camera_FOV and yaw must be in radians
def update_grid(grid, depth_map, camera_FOV, x, z, yaw):

    # TODO: Description
    # Calculate how far the obstacle is from the robot based on the yaw
    # and the given depth. Then, convert these distances to the absolute
    # x and y coordinates of the obstacle in the world, by adding to it
    # the robot's position.

    #   z-axis
    #   ^
    #   |
    #   |  dmap[0] -> \         }
    #   |   dmap[1] -> \        }
    #   |              .\       } obstacle wall (varying depth along axis)
    #   |             /  \      }
    #                /    \ <---}-- dmap[n]
    #               /
    #              /
    #             / ) <- yaw
    #            · ――――――――――――――――――――――――――――――――――> x-axis
    #            ^
    #            |
    #           x, z (robot position)
    #

    num_pixels = len(depth_map)  # Can be thought of as rays.
    pixel_FOV = camera_FOV / (num_pixels - 1)
    leftmost = - camera_FOV / 2
    rightmost = pixel_FOV - leftmost
    yaw_map = yaw + np.arange(leftmost, rightmost, pixel_FOV)

    # TODO: Description
    dx = np.cos(yaw_map) * depth_map  # NOTE: this is and must be the hadamard product.
    dz = np.sin(yaw_map) * depth_map

    obstacles_x = x + dx
    obstacles_z = z + dz
    print(obstacles_x)
    print(obstacles_z)

    # Convert the position of the obstacle to grid coordinates.
    grid_z = grid.shape[0] - 1
    obstacles_x = np.ceil(obstacles_x).astype(np.int32)
    obstacles_z = grid_z - np.ceil(obstacles_z).astype(np.int32)

    # NumPy and OpenCV begin indexing from the top-left corner, which means
    # that the robot's y must be adjusted to match grid coordinates.
    x = int(np.floor(x))
    z = int(np.floor(grid_z - z))

    print(obstacles_x)
    print(obstacles_z)

    # In NumPy, the first index selects the rows (y-axis),
    # while the second index selects the columns (x-axis).
    grid[obstacles_z][obstacles_x] = grid[obstacles_z][obstacles_x] + 1
    return grid

    # Create a box, large enough to fit the depth size, in any orientation,
    # and then draw the depth line on it. The resulting array will
    # be used to update all grid pixels up to where the depth ends -
    # that is, where the sensor has identified that an obstacle is present.
    box = np.zeros((-(obstacle_z - z), obstacle_x - x))

    # Draw the depth line on top of the box. This is equivalent
    # to setting to "1" all the pixels traversed by the depth line.
    # Unlike NumPy, in most OpenCV functions, the first dindex indicates
    # the Nth column (x-axis), and the second index the Nth row (y-axis).
    box_z = box.shape[0] - 1
    cv2.line(box, (0, box_z), (int(dx), box_z - int(dy)),
             color=1, thickness=1)

    # Negate the box to show that the space indicated by the line that
    # was drawn is empty (unoccupied).
    box = -box.astype(grid.dtype)

    # Finally, add the box to its corresponding area in the grid,
    # to indicate that all pixels traversed by the depth line
    # have been identified as empty.
    x1 = x
    x2 = obstacle_x
    z1 = obstacle_z + 1
    z2 = z + 1
    # TODO: Only works for square grids. Perhaps expand to rectangles.
    grid[z1:z2, x1:x2] = grid[z1:z2, x1:x2] + box

    return grid



##
grid = np.zeros((20, 20)).astype(np.int16)
depth_map = 5*np.ones(5)
grid = update_grid(grid, depth_map, np.pi, np.pi/4, 0, 0)

##
# Subscribed topics
# /dji_sdk/rtk_position
# ARGUMENTS
# radius
