import numpy as np
import cv2


class Grid:

    def __init__(self, radius, attitude_stream, position_stream, camera_FOV):
        length = self.radius2length(radius)
        self.grid = np.zeros((length, length), dtype=np.int16)

    # Convert the given radius into an appropriate computatonal length.
    def radius2length(self, radius):
        length = np.int(radius)  # TODO: adjust for relative size
        return length

    # TODO: description and parameters.
    # NOTE: camera_FOV and yaw must be in radians
    def update(self, depth_map, camera_FOV, yaw, x, z):

        # TODO: use self variables instead

        # Create a "yaw map" that corresponds every value in the depth map
        # to an angle that indicates where this depth can be found in the
        # camera's FOV (field of view). Think of the camera's view as a
        # collection of rays, all of which start from the optical center,
        # and eventually intersect a set of objects. The further an object
        # is from the optical center, the higher the length of the ray, and
        # therefore the higher its "depth". The depth_map array returns these
        # depths, starting from the leftmost ray of the camera's view, and up
        # until the rightmost. The yaw_map array stores the angle at which each
        # ray is oriented in comparison to the z-axis (the normal to the
        # imaging plane, starting from the optical center).
        # In other words, every ray `i` can be fully characterized as the
        # ray of length depth_map[i] at an angle yaw_map[i] from the normal.
        num_pixels = len(depth_map)
        pixel_FOV = camera_FOV / (num_pixels - 1)
        leftmost = - camera_FOV / 2
        rightmost = pixel_FOV - leftmost
        yaw_map = yaw + np.arange(leftmost, rightmost, pixel_FOV)

        # Then, "rotate" the depth map, such that the depths are given with
        # respect to the reference frame.

        #   z-axis
        #   ^
        #   |
        #   |  dmap[0] -> \\        }
        #   |   dmap[1] -> \\       }
        #   |              .\\      } obstacle wall (varying depth along axis)
        #   |             /  \\     }
        #                /    \\<---}-- dmap[n]
        #               /
        #              /
        #             / ) <- yaw
        #            · ――――――――――――――――――――――――――――――――――> x-axis
        #            ^
        #            |
        #           x, z (robot position)
        #

        # NOTE: In NumPy, * indicates a Hadamard product.
        o_dx = np.cos(yaw_map) * depth_map
        o_dz = np.sin(yaw_map) * depth_map
        depth_map = depth_map - 1
        u_dx = np.cos(yaw_map) * depth_map
        u_dz = np.sin(yaw_map) * depth_map

        o_x = np.ceil(x + o_dx).astype(np.int32)
        o_z = np.ceil(z + o_dz).astype(np.int32)
        u_x = np.ceil(x + u_dx).astype(np.int32)
        u_z = np.ceil(z + u_dz).astype(np.int32)
        # print(obstacles_x)
        # print(obstacles_z)

        # Convert the position of the obstacle to grid coordinates.
        # grid_z = self.grid.shape[0] - 1
        # obstacles_x = np.ceil(obstacles_x).astype(np.int32)
        # obstacles_z = np.ceil(obstacles_z).astype(np.int32)
        # obstacles_z = grid_z - np.ceil(obstacles_z).astype(np.int32)

        # NumPy and OpenCV begin indexing from the top-left corner, which means
        # that the robot's y must be adjusted to match grid coordinates.
        x = int(np.floor(x))
        z = int(np.floor(z))

        # print(obstacles_x)
        # print(obstacles_z)

        # Think of this as "ray-tracing":
        # given a point where the ray intersects an object, and an angle,
        # trace this ray to indicate
        max_depth = int(np.max(depth_map) + 1)
        box_len = 2*max_depth + 1
        box = np.zeros((box_len, box_len)).astype(self.grid.dtype)

        o_points = np.stack([o_x, o_z]).T
        o_points = o_points - np.array([x-max_depth, z-max_depth])
        o_points = np.vstack([np.array([max_depth, max_depth]), o_points])
        cv2.fillPoly(box, [o_points], -1, cv2.LINE_8)

        u_points = np.stack([u_x, u_z]).T
        u_points = u_points - np.array([x-max_depth, z-max_depth])
        u_points = np.vstack([np.array([max_depth, max_depth]), u_points])
        cv2.fillPoly(box, [u_points], 1, cv2.LINE_8)

        # print(points)
        # NOTE: 16 represents int16

        # print(box)
        # Edge case: on an edge.
        x1 = x - max_depth
        if x1 < 0:
            box = box[-x1:, :]
            x1 = 0
        x2 = x + max_depth + 1
        z1 = z - max_depth
        if z1 < 0:
            box = box[:, -z1:]
            z1 = 0
        z2 = z + max_depth + 1
        # self.grid[] = self.grid[] + box
        # print(self.grid[0:5, x1:x2])
        # print(x1, x2, z1, z2)
        self.grid[z1:z2, x1:x2] = self.grid[z1:z2, x1:x2] + box

        # print(points)
        # cv2.fillPoly(self.grid, [points], 1, 16)

        # In NumPy, the first index selects the rows (y-axis),
        # while the second index selects the columns (x-axis).
        # self.grid[obstacles_z][obstacles_x] = self.grid[obstacles_z][obstacles_x] + 1
        return self.grid


        # Create a box, large enough to fit the depth size, in any orientation,
        # and then draw the depth line on it. The resulting array will
        # be used to update all grid pixels up to where the depth ends -
        # that is, where the sensor has identified that an obstacle is present.
        # box = np.zeros((-(obstacle_z - z), obstacle_x - x))

        # Draw the depth line on top of the box. This is equivalent
        # to setting to "1" all the pixels traversed by the depth line.
        # Unlike NumPy, in most OpenCV functions, the first dindex indicates
        # the Nth column (x-axis), and the second index the Nth row (y-axis).
        # box_z = box.shape[0] - 1
        # cv2.line(box, (0, box_z), (int(dx), box_z - int(dy)),
                 # color=1, thickness=1)

        # Negate the box to show that the space indicated by the line that
        # was drawn is empty (unoccupied).
        # box = -box.astype(grid.dtype)

        # Finally, add the box to its corresponding area in the grid,
        # to indicate that all pixels traversed by the depth line
        # have been identified as empty.
        # x1 = x
        # x2 = obstacle_x
        # z1 = obstacle_z + 1
        # z2 = z + 1
        # TODO: Only works for square grids. Perhaps expand to rectangles.
        # grid[z1:z2, x1:x2] = grid[z1:z2, x1:x2] + box

        # return grid


##
grid = Grid(20)
depth_map = 7*np.ones(9)
# depth_map[0] = 9
# depth_map[1] = 0
# depth_map[2] = 0
# depth_map[3] = 0
# depth_map[4] = 0
# depth_map[5] = 0
# depth_map[6] = 0
# depth_map[7] = 0
# depth_map[7] = 4
camera_FOV = np.pi/2
yaw = np.pi/4
x = 3
z = 3
out = grid.update_grid(depth_map, camera_FOV, yaw, x, z)
print(out)
