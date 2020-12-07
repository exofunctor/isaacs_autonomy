#!/usr/bin/env python
# from camera_stream import CameraStream
import numpy as np
import cv2


class DepthMap:

    # TODO: change to attitude stream
    def __init__(self, disparity_stream, pitch_stream, roll_stream, f=None):
        # TODO return as grayscale only
        # self.camera_stream_topic = "/dji_sdk/stereo_240p_front_depth_images"
        # self.camera_stream = CameraStream(self.camera_stream_topic)
        self.disparity_stream = disparity_stream
        self.pitch_stream = pitch_stream
        self.roll_stream = roll_stream

        # The image dimensions.
        self.h = self.disparity_stream().shape[0]
        self.w = self.disparity_stream().shape[1]

        # The focal length.
        self.d = np.sqrt(self.h**2 + self.w**2)
        self.f = f

    def callback(self):
        disparity = self.disparity_stream()
        pitch = self.pitch_stream()
        roll = self.roll_stream()
        # TODO: Description.
        depth_map = self.warp3D(disparity, pitch, roll)
        # Reduce the depth map to a 1D array.
        depth_map = self.maxpool_columns(depth_map)
        return depth_map

    # Warp an image using a 3D-Rotational model.
    # TODO: description
    # TODO: find focal_length
    # NOTE: im must be b&w
    def warp3D(self, im, pitch=0, roll=0):

        # We pass in negative angle values, because we want to
        # "undo" the rotations such that the rows of the disparity
        # image are parallel to the xz-plane.
        cx = np.cos(-pitch)
        sx = np.sin(-pitch)
        cz = np.cos(-roll)
        sz = np.sin(-roll)

        # If the focal length is not known, estimate an ideal focal length.
        if self.f is None:
            f = self.d / (2 * sz if sz != 0 else 1)
        else:
            f = self.f

        # The camera 2D-to-3D-Projection Matrix.
        X = np.array([
                     [1,         0, -self.w/2],
                     [0,         1, -self.h/2],
                     [0,         0,         1],
                     [0,         0,         1]
                     ])

        # The Matrix that "undoes" the rotation around the x-axis.
        Rx = np.array([
                     [1,         0,         0,         0],
                     [0,        cx,       -sx,         0],
                     [0,        sx,        cx,         0],
                     [0,         0,         0,         1]
                     ])

        # The Matrix that "undoes" the rotation around the z-axis.
        Rz = np.array([
                     [cz,      -sz,         0,         0],
                     [sz,       cz,         0,         0],
                     [0,         0,         1,         0],
                     [0,         0,         0,         1]
                      ])

        # The "total" 3D-Rotation Matrix.
        R = np.matmul(Rx, Rz)

        # The Translation Matrix.
        T = np.identity(4)
        T[2][3] = f

        # The camera Intrinsic Matrix.
        K = np.array([
                     [f,         0,  self.w/2,         0],
                     [0,         f,  self.h/2,         0],
                     [0,         0,         1,         0]
                     ])

        # The Perspective Transformation Matrix.
        perspective = np.matmul(K, np.matmul(T, np.matmul(R, X)))

        # NOTE: The interpolation can be changed to INTER_LINEAR if too slow.
        im = cv2.warpPerspective(im, perspective, (self.w, self.h),
                                 flags=cv2.INTER_LANCZOS4)

        return im

    # Return the highest value of each colun of the given image.
    # If `im` is a 2D depth map, this reduces it down to a 1D array of depths.
    def maxpool_columns(self, im):
        return np.max(im, axis=0)


#####
im = imread("test.jpg")
im0 = im[:, :, 0]
disparity_stream = lambda: im0
pitch_stream = lambda: np.pi/4
roll_stream = lambda: 0
depth_map = DepthMap(disparity_stream, pitch_stream, roll_stream)
out = depth_map.warp3D(disparity_stream(), pitch_stream(), roll_stream())
imshow(out)
# maxout = depth_map.maxpool_columns(out)
# print(maxout)
# imshow(maxout)
#####
