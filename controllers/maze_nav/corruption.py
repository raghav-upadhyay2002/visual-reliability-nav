"""Visual corruption conditions for reliability evaluation.

Applied identically to all three camera frames (front/left/right) right after
capture in maze_nav.py, before any detector sees them -- so every detector
degrades together the way a real degraded sensor would, rather than only
corrupting the one driving wall_ahead. Controlled by env vars so the (not yet
built) eval driver can sweep corruption type x severity across many trials
without touching controller code:

  MAZE_NAV_CORRUPTION           -- one of CORRUPTION_TYPES, default 'clean'
  MAZE_NAV_CORRUPTION_SEVERITY  -- int 1-5, default 1 (ignored for 'clean')

Severity follows the ImageNet-C convention (Hendrycks & Dietterich): 5
discrete levels of increasing intensity per corruption type. The specific
per-level parameter values below are a first-pass calibration by eye (chosen
so level 1 is mild and level 5 is severe), not yet validated against what
actually degrades this project's detectors -- that validation is exactly
what the eval sweep this module supports is for.
"""

import os

import cv2
import numpy as np

CORRUPTION_TYPES = [
    'clean',
    'low_illumination',
    'blur',
    'occlusion',
    'noise',
    'brightness_contrast',
    'reduced_fov',
]

MIN_SEVERITY = 1
MAX_SEVERITY = 5


def _low_illumination(img_bgr, severity):
    """Scale down the HSV value channel -- simulates a dim/low-light scene.

    Working in HSV (rather than scaling BGR directly) keeps hue/saturation
    intact so this reads as "darker", not "differently colored".
    """
    factor = [0.8, 0.6, 0.45, 0.3, 0.15][severity - 1]
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 2] *= factor
    hsv = np.clip(hsv, 0, 255).astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def _blur(img_bgr, severity):
    """Gaussian blur -- simulates defocus/motion blur. Kernel size scales with severity."""
    ksize = [3, 5, 9, 13, 17][severity - 1]
    return cv2.GaussianBlur(img_bgr, (ksize, ksize), 0)


def _occlusion(img_bgr, severity):
    """Black out a corner patch -- simulates a lens smudge/sticker/dead sensor region.

    Fixed top-left position (not randomized per frame) so it behaves like a
    static physical defect for the whole trial, not per-frame flicker.
    """
    frac = [0.1, 0.2, 0.35, 0.5, 0.65][severity - 1]
    height, width = img_bgr.shape[:2]
    patch_h, patch_w = int(height * frac), int(width * frac)
    out = img_bgr.copy()
    out[0:patch_h, 0:patch_w] = 0
    return out


def _noise(img_bgr, severity):
    """Additive Gaussian pixel noise -- simulates sensor/readout noise."""
    std = [10, 20, 35, 50, 70][severity - 1]
    noise = np.random.normal(0, std, img_bgr.shape)
    return np.clip(img_bgr.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def _brightness_contrast(img_bgr, severity):
    """Blend toward a bright near-white value -- simulates overexposed/washed-out/hazy
    imaging. The complementary failure direction to _low_illumination's darkening.

    new = alpha*pixel + (1-alpha)*HAZE_TARGET, so pixels are pulled toward a
    light haze color as severity rises (alpha falls) rather than toward an
    arbitrary offset -- a naive `alpha*pixel + beta` with small beta actually
    darkens bright pixels (e.g. a 230-value floor pixel), which reads as dim,
    not washed-out.
    """
    HAZE_TARGET = 235
    alpha = [0.85, 0.7, 0.55, 0.4, 0.25][severity - 1]
    beta = (1 - alpha) * HAZE_TARGET
    return cv2.convertScaleAbs(img_bgr, alpha=alpha, beta=beta)


def _reduced_fov(img_bgr, severity):
    """Center-crop then resize back up -- approximates a narrower field of view.

    Not a true optical FOV change -- that would mean editing the Camera node's
    fieldOfView field per world and re-rendering, which can't be swept
    per-trial at runtime. This is a post-hoc digital-zoom proxy: crop out the
    periphery, then upscale the remaining center back to the original
    resolution, so downstream detectors see less of the scene per frame the
    same way a physically narrower lens would.
    """
    keep = [0.85, 0.7, 0.55, 0.4, 0.25][severity - 1]
    height, width = img_bgr.shape[:2]
    crop_h, crop_w = int(height * keep), int(width * keep)
    y0, x0 = (height - crop_h) // 2, (width - crop_w) // 2
    cropped = img_bgr[y0:y0 + crop_h, x0:x0 + crop_w]
    return cv2.resize(cropped, (width, height), interpolation=cv2.INTER_LINEAR)


_CORRUPTIONS = {
    'low_illumination': _low_illumination,
    'blur': _blur,
    'occlusion': _occlusion,
    'noise': _noise,
    'brightness_contrast': _brightness_contrast,
    'reduced_fov': _reduced_fov,
}


def apply_corruption(img_bgr, corruption_type, severity):
    """Apply one named corruption at one severity level (1-5) to a BGR frame.
    'clean' (or None) returns the frame unchanged."""
    if corruption_type is None or corruption_type == 'clean':
        return img_bgr
    if corruption_type not in _CORRUPTIONS:
        raise ValueError(f"unknown corruption type: {corruption_type!r} (expected one of {CORRUPTION_TYPES})")
    if not (MIN_SEVERITY <= severity <= MAX_SEVERITY):
        raise ValueError(f"severity must be {MIN_SEVERITY}-{MAX_SEVERITY}, got {severity}")
    return _CORRUPTIONS[corruption_type](img_bgr, severity)


def read_corruption_config():
    """Read MAZE_NAV_CORRUPTION / MAZE_NAV_CORRUPTION_SEVERITY from the environment.
    Defaults to ('clean', 1) -- i.e. no corruption -- when unset, so existing
    interactive runs are unaffected unless these are explicitly set."""
    corruption_type = os.environ.get('MAZE_NAV_CORRUPTION', 'clean')
    severity = int(os.environ.get('MAZE_NAV_CORRUPTION_SEVERITY', '1'))
    return corruption_type, severity
