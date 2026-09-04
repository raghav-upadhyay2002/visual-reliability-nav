"""Camera-frame -> navigation-signal detectors (target color, wall edges/brightness).

This is the edge-density / brightness approach actually used by the
controller's main loop. classic_cv.py holds a separate contour/Hough-line
approach that isn't currently wired in.
"""

import cv2
import numpy as np

from config import (
    WALL_DENSITY_EPSILON,
    WALL_CONSECUTIVE_FRAMES,
    RIGHT_WALL_BRIGHTNESS_THRESHOLD,
)


def detect_target(img_bgr):
    """Look for the red target in a BGR camera frame.

    Returns (found, direction, mask) where direction is 'left'/'center'/'right'
    (None if not found), and mask is the binary image of pixels classified as red.
    """
    # HSV separates color (hue) from lighting (value), which makes
    # thresholding a specific color far more reliable than in raw BGR.
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Red wraps around the hue wheel (near 0 and near 180), so two ranges
    # are needed to catch both ends and cover the full red band.
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([7, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # inRange() gives a binary mask of pixels inside each range; combining
    # them covers both halves of the red hue band.
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Number of red pixels found; used as a simple "is the target visible" threshold.
    pixel_count = cv2.countNonZero(mask)

    if pixel_count > 50:
        # Average x-position of every red pixel gives the target's horizontal
        # center in the frame, which tells us which way to steer.
        ys, xs = np.where(mask > 0)
        cx = np.mean(xs)
        width = img_bgr.shape[1]

        if cx < width * 0.4:
            direction = 'left'
        elif cx > width * 0.6:
            direction = 'right'
        else:
            direction = 'center'

        return True, direction, mask
    else:
        return False, None, mask


def detect_edges(img_bgr):
    """Grayscale + blur + Canny -> a binary edge map of the frame."""
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # Smooth out noise first so Canny doesn't pick up false edges from grain/texture.
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # Canny returns a binary image: white pixels mark detected edges.
    edges = cv2.Canny(blurred, 50, 150)
    return edges


class WallDetector:
    """Estimate wall presence ahead/left/right from the front camera's edge density.

    Tracks consecutive-frame counters internally (one instance per robot/trial)
    so a wall must be seen for several frames in a row before it's reported,
    which debounces single-frame noise.
    """

    def __init__(self):
        self._center_count = 0
        self._left_count = 0
        self._right_count = 0

    def update(self, img_bgr):
        edges = detect_edges(img_bgr)
        height, width = edges.shape

        # Split the frame into left/center/right thirds so wall presence can
        # be judged separately in each direction the robot could turn toward.
        left_zone = edges[:, 0:int(width * 0.33)]
        center_zone = edges[:, int(width * 0.33):int(width * 0.66)]
        right_zone = edges[:, int(width * 0.66):]

        # Fraction of edge pixels in each zone. Note the signal is inverted
        # from what you might expect: a wall close enough to matter fills the
        # zone as a flat, low-texture surface and produces almost NO edges,
        # while open space (floor, sky, distant scenery) always retains
        # contrast and produces many. See RESULTS.md, Problem 1 -- background
        # scenery makes "many edges = wall" unusable.
        left_density = cv2.countNonZero(left_zone) / left_zone.size
        center_density = cv2.countNonZero(center_zone) / center_zone.size
        right_density = cv2.countNonZero(right_zone) / right_zone.size

        eps = WALL_DENSITY_EPSILON

        # Wall directly ahead: all three zones read as flat/close.
        if left_density < eps and center_density < eps and right_density < eps:
            self._center_count += 1
        else:
            self._center_count = 0
        wall_ahead = self._center_count >= WALL_CONSECUTIVE_FRAMES

        # Wall hugging the left side, with an opening to the right.
        if left_density < eps and right_density > eps:
            self._left_count += 1
        else:
            self._left_count = 0
        wall_left = self._left_count >= WALL_CONSECUTIVE_FRAMES

        # Wall hugging the right side, with an opening to the left.
        if right_density < eps and left_density > eps:
            self._right_count += 1
        else:
            self._right_count = 0
        wall_front_right = self._right_count >= WALL_CONSECUTIVE_FRAMES

        return {
            'wall_ahead': wall_ahead,
            'wall_left': wall_left,
            'wall_front_right': wall_front_right,
            'left_density': left_density,
            'center_density': center_density,
            'right_density': right_density,
            'edges': edges,
        }


def bottom_img_right(img_bgr_right, split_ratio):
    """Crop the bottom fraction (split_ratio) of the right camera's frame.

    The right side camera is aimed along the wall the robot follows, so the
    bottom strip of its image is what's closest to (and most informative about)
    that wall.
    """
    height, width = img_bgr_right.shape[:2]
    split_y = int(height * (1 - split_ratio))
    return img_bgr_right[split_y:, :]


def detect_walls_status_right(img_bgr_right, split_ratio):
    """Detect a wall on the right from how dark the bottom strip of that camera is.

    A nearby wall fills the frame and reduces the average brightness compared
    to open floor, so a mean-brightness threshold suffices at baseline illumination.
    """
    img_bottom_right = bottom_img_right(img_bgr_right, split_ratio)
    gray_right = cv2.cvtColor(img_bottom_right, cv2.COLOR_BGR2GRAY)
    mean_right = np.mean(gray_right)

    wall_right = mean_right < RIGHT_WALL_BRIGHTNESS_THRESHOLD

    return {
        'wall_right': wall_right,
        'mean_right': mean_right,
        'img_bottom_right': img_bottom_right,
    }
