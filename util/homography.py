import numpy as np
import cv2


# Create the 3xN meshgrid corresponding to `im`.
# This meshgrid will be an array of the form:
#    0 1 2     0 1 2     0 1 2     im.width  (x)
#    0 0 0 ... 1 1 1 ... 2 2 2 ... im.height (y)
#    1 1 1     1 1 1     1 1 1          1
def create_meshgrid(im):
    """
    - im: a 2D or 3D NumPy array representing an image.
    """
    x = np.arange(im.shape[1])
    y = np.arange(im.shape[0])
    xx, yy = np.meshgrid(x, y)
    ones = np.ones(im.shape[:2])

    # This creates a meshrig in the shape of `im`.
    meshgrid = np.stack((xx, yy, ones), axis=1)

    # This reshapes the meshgrid to be 3xN.
    x = meshgrid[:, 0, :]
    y = meshgrid[:, 1, :]
    meshgrid = np.array([x.ravel(), y.ravel(), np.ones_like(x).ravel()])

    return meshgrid


# Warp an image into another, per the specified homography.
# The algorithm uses inverse warping with bilinear interpolation.
# Forward warping will generally reduce quality. Other types of
# interpolation, such as Lanczos, will work, but may be slower.
def warp_H(im, dst, H):
    """
    - im:  a 2D or 3D NumPy array representing an image.
    - dst: a 2D or 3D NumPy array corresponding to the space where
           `im` should be warped to.
    - H:   a 3x3 matrix representing the homography that should be applied
           to go from `im` to `dst`.
    """

    # Get a meshgrid correpsonding to the discrete
    # coordinate values of the destination space.
    meshgrid = create_meshgrid(dst)

    # Invert the homography, to use the inverse warping.
    H = np.linalg.pinv(H)

    # Apply the inverted homography to the mehsgrid, to compute
    # the position that each discrete block (aka. pixel) in the
    # destination space maps to in the image space (`im`).
    meshgrid = np.matmul(H, meshgrid)

    # Define the x and y maps for the interpolation function.
    map_x, map_y = meshgrid[:-1] / meshgrid[-1]
    h, w = dst.shape[:2]
    map_x = map_x.reshape(h, w).astype(np.float32)
    map_y = map_y.reshape(h, w).astype(np.float32)

    # Interpolate the Homography-transformed meshgrid to get the
    # color corresponding to each pixel in the transformed image.
    im_out = cv2.remap(im, map_x, map_y, cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT)
    im_out = np.clip(im_out, 0., 1.)

    return im_out
