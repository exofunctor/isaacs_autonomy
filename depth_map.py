#!/usr/bin/env python
import numpy as np
import cv2


# TODO TODO: find focal_length
class DepthMap:

    def __init__(self):

        # The current depth map.
        self.depth_map = None

    # TODO: Description.
    def update(self, disparity, pitch, roll, f=None, verbose=False):
        # TODO TODO: convert disparity to depth measure.
        # TODO TODO: use altitude measurement
        # Warp the depth map such that it is parallel to the
        # xz-plane that we are searching in.
        # depth_map = disparity

        disparity /= 255.
        depth_map = self.warp3D(disparity, pitch, roll, f)

        if verbose:
            disparity_gauss1 = cv2.GaussianBlur(disparity, 25, 10)
            disparity_gauss2 = cv2.GaussianBlur(disparity, 5, 30)
            depth_map1 = self.warp3D(disparity_gauss1, pitch, roll, f)
            depth_map2 = self.warp3D(disparity_gauss2, pitch, roll, f)

            top = np.hstack([disparity, disparity_gauss1, disparity_gauss2])
            bottom = np.hstack([depth_map, depth_map1, depth_map2])
            out = np.vstack([top, bottom])

            # cv2.imshow("Depth Map", depth_map)
            cv2.imshow("Depth Perception", out)
            cv2.waitKey(3)

        # Reduce the depth map to a 1D array.
        depth_map = self.maxpool_columns(depth_map)
        self.depth_map = depth_map
        return self.depth_map

    # Warp an image using a 3D-Rotational model.
    # TODO: sketch
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

        # NOTE: The interpolation can be changed to INTER_LANCZOS4
        # for better quality if speed is not an issue.
        im = cv2.warpPerspective(im, perspective, (w, h),
                                 flags=cv2.INTER_LINEAR)

        return im

    # Return the highest value of each colun of the given image.
    # If `im` is a 2D depth map, this reduces it down to a 1D array of depths.
    def maxpool_columns(self, im):
        # TODO: select only part of the image.
        return np.max(im, axis=0)


#####
# im = imread("test.jpg")
# im0 = im[:, :, 0]
# disparity_stream = lambda: im0
# pitch_stream = lambda: np.pi/4
# roll_stream = lambda: 0
# depth_map = DepthMap(disparity_stream, pitch_stream, roll_stream)
# out = depth_map.warp3D(disparity_stream(), pitch_stream(), roll_stream())
# imshow(out)
# maxout = depth_map.maxpool_columns(out)
# print(maxout)
# imshow(maxout)
#####
