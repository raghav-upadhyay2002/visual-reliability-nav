from controller import Robot
import numpy as np
import cv2
import csv
from classic_cv import detect_walls_countours, detect_walls_lines

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Debounce counters: a zone must show a wall for several consecutive frames
# before it's trusted, same idea as the old edge-density approach.
center_wall_count = 0
left_wall_count = 0
right_wall_count = 0

# Cameras

# camera_front
front_camera = robot.getDevice('camera')
front_camera.enable(timestep)
print("Front Camera resolution: ", front_camera.getWidth(), "x", front_camera.getHeight())


# motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


def zone_wall_flags(walls, width, min_area=80):
    """Split the frame into left/center/right thirds and flag which zone(s)
    contain a wall blob large enough to trust (from detect_walls_countours)."""
    left = center = right = False
    for w in walls:
        if w["area"] < min_area:
            continue
        x, _, bw, _ = w["bbox"]
        cx = x + bw / 2
        if cx < width * 0.33:
            left = True
        elif cx > width * 0.66:
            right = True
        else:
            center = True
    return left, center, right


# main loop
while robot.step(timestep) != -1:

    # raw image from front camera
    front_image = front_camera.getImage()
    width = front_camera.getWidth()
    height = front_camera.getHeight()

    # reshape the flat buffer to a 3D array (height, width, channels)
    front_image = np.frombuffer(front_image, np.uint8).reshape((height, width, 4))

    # drop the alpha channel
    front_image_bgr = cv2.cvtColor(front_image, cv2.COLOR_BGRA2BGR)

    # Pipeline A (contours) drives navigation: wall blobs per zone.
    walls, debug_a = detect_walls_countours(front_image_bgr)
    left_wall, center_wall, right_wall = zone_wall_flags(walls, width)

    # Pipeline B (Hough lines) runs alongside for comparison/visualization only
    # -- not driving motor decisions yet, since combining both into one policy
    # is a separate design choice.
    lines, debug_b = detect_walls_lines(front_image_bgr)

    center_wall_count = center_wall_count + 1 if center_wall else 0
    left_wall_count = left_wall_count + 1 if left_wall else 0
    right_wall_count = right_wall_count + 1 if right_wall else 0

    wall_ahead = center_wall_count >= 3
    wall_left = left_wall_count >= 3
    wall_right = right_wall_count >= 3

    # Navigation priority: avoid walls first, otherwise drive straight.
    if wall_ahead:
        if wall_right:
            # Right side also blocked -> turn left.
            left_velocity = 0.0
            right_velocity = 3.0
        else:
            # Right side open -> turn right.
            left_velocity = 3.0
            right_velocity = 0.0
    elif wall_left:
        # Wall hugging the left -> steer away from it (turn right).
        left_velocity = 3.0
        right_velocity = 0.0
    elif wall_right:
        # Wall hugging the right -> steer away from it (turn left).
        left_velocity = 0.0
        right_velocity = 3.0
    else:
        left_velocity = 3.0
        right_velocity = 3.0

    left_motor.setVelocity(left_velocity)
    right_motor.setVelocity(right_velocity)

    print("wall_ahead:", wall_ahead, "wall_left:", wall_left, "wall_right:", wall_right,
          "| blobs:", len(walls), "lines:", len(lines))

    # show the image in a window
    cv2.imshow("Front Camera", cv2.resize(front_image_bgr, (300, 300)))
    cv2.imshow("Pipeline A - result", debug_a["result"])
    cv2.imshow("Pipeline B - result", debug_b["result"])

    # required for Open cv to refresh the window
    cv2.waitKey(1)


# close the window when the simulation ends
cv2.destroyAllWindows()
