#!/usr/bin/env python
import numpy as np
import cv2


class DepthMap:

    def __init__(self):

        # The current depth map.
        self.depth_map = None

    # Receive a disparity map and convert it to a usable depth measure.
    def update(self, disparity, pitch, roll, f=None):

        # Median blur the disparity image to
        disparity = cv2.medianBlur(disparity, 5)

        # Convert the disparity dtype to float32.
        disparity = disparity/255.

        # Warp the depth map such that it is parallel to the
        # xz-plane that we are searching in.
        depth_map = self.warp3D(disparity, pitch, roll, f)

        # Reduce the depth map to a 1D array.
        depth_map = self.maxpool_columns(depth_map)
        self.depth_map = depth_map
        return self.depth_map

    # Warp an image using a 3D-Rotational model.
    def warp3D(self, im, pitch, roll, f=None):

        # The depth image dimensions.
        h = im.shape[0]
        w = im.shape[1]

        # We pass in negative angle values, because we want to
        # "undo" the rotations such that the rows of the disparity
        # image are parallel to the xz-plane.
        cx = np.cos(-pitch)
        sx = np.sin(-pitch)
        cz = np.cos(-roll)
        sz = np.sin(-roll)

        # If the focal length is not known, estimate an ideal focal length.
        if f is None:
            d = np.sqrt(h**2 + w**2)
            f = d / (2 * sz if sz != 0 else 1)

        # The camera 2D-to-3D-Projection Matrix.
        X = np.array([
                     [1,    0, -w/2],
                     [0,    1, -h/2],
                     [0,    0,    1],
                     [0,    0,    1]
                     ])

        # The Matrix that "undoes" the rotation around the x-axis.
        Rx = np.array([
                     [1,    0,    0,    0],
                     [0,   cx,  -sx,    0],
                     [0,   sx,   cx,    0],
                     [0,    0,    0,    1]
                     ])

        # The Matrix that "undoes" the rotation around the z-axis.
        Rz = np.array([
                     [cz, -sz,    0,    0],
                     [sz,  cz,    0,    0],
                     [0,    0,    1,    0],
                     [0,    0,    0,    1]
                     ])

        # The "total" 3D-Rotation Matrix.
        R = np.matmul(Rx, Rz)

        # The Translation Matrix.
        T = np.identity(4)
        T[2][3] = f

        # The camera Intrinsic Matrix.
        K = np.array([
                     [f,    0,  w/2,    0],
                     [0,    f,  h/2,    0],
                     [0,    0,    1,    0]
                     ])

        # The Perspective Transformation Matrix.
        perspective = np.matmul(K, np.matmul(T, np.matmul(R, X)))

        # The interpolation can be changed to INTER_LANCZOS4
        # for better quality if speed is not an issue.
        im = cv2.warpPerspective(im, perspective, (w, h),
                                 flags=cv2.INTER_LINEAR)

        return im

    # Return the highest value of each colun of the given image.
    # If `im` is a 2D depth map, this reduces it down to a 1D array of depths.
    # `pct` indicates what percentage from the center of the iamge should be
    # used (the rest will be discarded).
    def maxpool_columns(self, im, pct=.4):
        dh = int(pct * im.shape[0])
        h1 = int(0.5 * (1 - pct) * im.shape[0])
        h2 = h1 + dh
        return np.max(im[h1:h2, :], axis=0)
