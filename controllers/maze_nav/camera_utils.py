"""Helpers for pulling frames off Webots camera devices."""

import cv2
import numpy as np

from config import BLUR_ENABLED, BLUR_SIZE


def blur_cam(img_bgr):
    if not BLUR_ENABLED:
        return img_bgr

    return cv2.GaussianBlur(img_bgr, (BLUR_SIZE, BLUR_SIZE), 0)


def get_camera_bgr(camera):
    """Grab a Webots camera's current frame as a standard 3-channel BGR image.

    Raw camera images come back as a flat byte buffer in BGRA order; this
    reshapes it and drops the alpha channel.
    """
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    img_bgra = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    return blur_cam(cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR))
