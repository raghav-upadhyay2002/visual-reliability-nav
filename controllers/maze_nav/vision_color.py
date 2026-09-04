"""Color-based wall detection: replaces vision.WallDetector (front camera,
Canny edge density) and vision.detect_walls_status_right (right camera,
absolute brightness).

Why: both of those depended on scene contrast (shading gradients, shadows)
that no longer exists now that every surface is a flat, unshaded
roughness=1/metalness=0 material with castShadows FALSE -- see the
maze_nav.py module docstring / project history for the diagnosis. Every
wall now renders as one constant color, which the edge detector can't see
at all, and the right-camera brightness threshold was never validated
against actual rendered output (observed means ran 30-120 against a
threshold of 195, latching "wall" permanently).

The fix keys off each maze's *actual* wall color (read from customData --
see world_meta.py) instead of a fixed absolute threshold, so it generalizes
across mazes with different wall/floor color schemes rather than being
tuned to one. Output dict keys match vision.py's detectors exactly
(wall_ahead/wall_left/wall_front_right/left_density/center_density/
right_density/edges, and wall_right/mean_right/img_bottom_right) so
trial_logger's CSV schema is unchanged -- the *_density/mean_right columns
now hold wall-color-fraction / mean-hue-match values instead, not density or
brightness; see trial_logger.LOG_HEADER's comment.

Untested assumption, flagged for whoever tunes this next: WALL_FRAC_THRESHOLD
below is a starting guess (a zone is "close" once ~45% of it is wall-colored),
not yet validated against a real run the way the old thresholds eventually
were. Needs the front camera actually pitched down (camera_rotation in the
world files) for wall_frac to scale with distance at all -- level-mounted at
close range it saturates near 1.0 regardless of true distance. Tuned against
a 0.15m-cell maze; cells are now 0.5m (see generate_maze_world.DEFAULT_CELL),
which changes how quickly wall_frac ramps as the robot approaches a wall --
not yet re-checked at the new scale.
"""

import cv2
import numpy as np

from config import WALL_CONSECUTIVE_FRAMES

WALL_FRAC_THRESHOLD = 0.45
HUE_TOLERANCE = 12
MIN_SATURATION = 40


def _rgb_to_hue(rgb):
    """Webots baseColor (r,g,b) floats in [0,1] -> OpenCV hue (0-179)."""
    r, g, b = rgb
    bgr = np.uint8([[[int(b * 255), int(g * 255), int(r * 255)]]])
    return int(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0][0])


def _color_mask(img_bgr, hue, tolerance=HUE_TOLERANCE, min_saturation=MIN_SATURATION):
    """Pixels within `tolerance` of `hue` and reasonably saturated (excludes
    the floor, which is a desaturated near-white/near-black in every world).
    Handles hue wraparound near 0/179 since hue is circular."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lo, hi = hue - tolerance, hue + tolerance

    if lo < 0:
        mask = cv2.inRange(hsv, (0, min_saturation, 0), (hi, 255, 255))
        mask |= cv2.inRange(hsv, (180 + lo, min_saturation, 0), (179, 255, 255))
    elif hi > 179:
        mask = cv2.inRange(hsv, (lo, min_saturation, 0), (179, 255, 255))
        mask |= cv2.inRange(hsv, (0, min_saturation, 0), (hi - 180, 255, 255))
    else:
        mask = cv2.inRange(hsv, (lo, min_saturation, 0), (hi, 255, 255))
    return mask


class ColorWallDetector:
    """Estimate wall presence ahead/left/right from what fraction of each
    front-camera zone matches this maze's own wall color.

    Same debounce/zone-split/decision structure as vision.WallDetector, just
    with the underlying per-zone signal swapped from edge density to
    wall-color fraction (and the polarity flipped: HIGH fraction = close,
    where density was LOW = close).
    """

    def __init__(self, wall_color_rgb, frac_threshold=WALL_FRAC_THRESHOLD):
        self.wall_hue = _rgb_to_hue(wall_color_rgb)
        self.frac_threshold = frac_threshold
        self._center_count = 0
        self._left_count = 0
        self._right_count = 0

    def update(self, img_bgr):
        mask = _color_mask(img_bgr, self.wall_hue)
        height, width = mask.shape

        left_zone = mask[:, 0:int(width * 0.33)]
        center_zone = mask[:, int(width * 0.33):int(width * 0.66)]
        right_zone = mask[:, int(width * 0.66):]

        left_frac = cv2.countNonZero(left_zone) / left_zone.size
        center_frac = cv2.countNonZero(center_zone) / center_zone.size
        right_frac = cv2.countNonZero(right_zone) / right_zone.size

        t = self.frac_threshold

        if left_frac > t and center_frac > t and right_frac > t:
            self._center_count += 1
        else:
            self._center_count = 0
        wall_ahead = self._center_count >= WALL_CONSECUTIVE_FRAMES

        if left_frac > t and right_frac < t:
            self._left_count += 1
        else:
            self._left_count = 0
        wall_left = self._left_count >= WALL_CONSECUTIVE_FRAMES

        if right_frac > t and left_frac < t:
            self._right_count += 1
        else:
            self._right_count = 0
        wall_front_right = self._right_count >= WALL_CONSECUTIVE_FRAMES

        return {
            'wall_ahead': wall_ahead,
            'wall_left': wall_left,
            'wall_front_right': wall_front_right,
            'left_density': left_frac,
            'center_density': center_frac,
            'right_density': right_frac,
            'edges': mask,
        }


def bottom_img_right(img_bgr_right, split_ratio):
    height, width = img_bgr_right.shape[:2]
    split_y = int(height * (1 - split_ratio))
    return img_bgr_right[split_y:, :]


def detect_walls_status_right_color(img_bgr_right, wall_hue, split_ratio, frac_threshold=WALL_FRAC_THRESHOLD):
    """Right-camera equivalent of ColorWallDetector: wall-color fraction of
    the bottom strip, instead of its mean brightness."""
    img_bottom_right = bottom_img_right(img_bgr_right, split_ratio)
    mask = _color_mask(img_bottom_right, wall_hue)
    wall_frac = cv2.countNonZero(mask) / mask.size

    return {
        'wall_right': wall_frac > frac_threshold,
        'mean_right': wall_frac,
        'img_bottom_right': img_bottom_right,
    }
