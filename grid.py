#!/usr/bin/env python
import numpy as np
import cv2


class Grid:

    def __init__(self, radius):
        self.radius = radius
        radius = int(2 * radius)
        self.grid = np.zeros((radius, radius), dtype=np.int16)

    # Important: `camera_FOV` and `yaw` must be in radians
    def update(self, depth_map, camera_FOV, yaw, x, z):

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
        #            . ----------------------------------> x-axis
        #            ^
        #            |
        #           x, z (robot position)
        #

        # In NumPy, * indicates a Hadamard product.
        o_dx = np.cos(yaw_map) * depth_map
        o_dz = np.sin(yaw_map) * depth_map
        depth_map = depth_map - 1
        u_dx = np.cos(yaw_map) * depth_map
        u_dz = np.sin(yaw_map) * depth_map

        # Convert the position of the obstacle to grid coordinates.
        # `o` stands for occupied, `u` stands for unoccupied.
        o_x = np.ceil(x + o_dx).astype(np.int32)
        o_z = np.ceil(z + o_dz).astype(np.int32)
        u_x = np.ceil(x + u_dx).astype(np.int32)
        u_z = np.ceil(z + u_dz).astype(np.int32)

        # NumPy and OpenCV begin indexing from the top-left corner.
        x = int(np.floor(x))
        z = int(np.floor(z))

        # Given a point where the ray intersects an object, and an angle,
        # trace this ray to indicate that the space until the object is free.
        max_depth = int(np.max(depth_map) + 1)
        box_len = 2*max_depth + 1
        box = np.zeros((box_len, box_len)).astype(self.grid.dtype)

        # Trace the obstacles.
        o_points = np.stack([o_x, o_z]).T
        o_points = o_points - np.array([x-max_depth, z-max_depth])
        o_points = np.vstack([np.array([max_depth, max_depth]), o_points])
        cv2.fillPoly(box, [o_points], -1, cv2.LINE_8)

        # "Project" the quadrotor's FOV on the grid by raytracing it.
        u_points = np.stack([u_x, u_z]).T
        u_points = u_points - np.array([x-max_depth, z-max_depth])
        u_points = np.vstack([np.array([max_depth, max_depth]), u_points])
        cv2.fillPoly(box, [u_points], 1, cv2.LINE_8)

        # Edge case: do not raytrace areas outside of the grid.
        x1 = x - max_depth
        if x1 < 0:
            box = box[:, -x1:]
            x1 = 0
        x2 = x + max_depth + 1
        z1 = z - max_depth
        if z1 < 0:
            box = box[-z1:, :]
            z1 = 0
        z2 = z + max_depth + 1

        # Update the grid by adding to it the box in which the quadrotor's FOV
        # was projected, and the map was raytraced.
        self.grid[z1:z2, x1:x2] = self.grid[z1:z2, x1:x2] + box

        return self.grid
