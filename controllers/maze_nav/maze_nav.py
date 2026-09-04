"""Webots controller: vision-based maze navigation with eval logging.

Reads the front/left/right cameras each step, derives wall/target detections
(vision.py), turns those into wheel velocities (navigation.py), and logs a
CSV row per step (trial_logger.py) for offline reliability analysis. Trial
termination (success/collision/timeout) is supervisor/sensor-driven and
never feeds back into the navigation decisions themselves.
"""

import math

from controller import Supervisor
import cv2

from config import TARGET_RADIUS, COLLISION_THRESHOLD, MAX_TRIAL_SECONDS, RIGHT_CAMERA_SPLIT_RATIO
from camera_utils import get_camera_bgr
from vision import detect_target, detect_walls_status_right, WallDetector
from navigation import decide_velocities
from trial_logger import TrialLogger


def main():
    # Webots hands us a Supervisor instance representing this simulation; all
    # devices (camera, motors, sensors) are accessed through it via getDevice().
    robot = Supervisor()

    # Simulation step size in ms. Every device must be enabled with this
    # value, and robot.step(timestep) must be called once per control loop
    # iteration to advance the simulation clock.
    timestep = int(robot.getBasicTimeStep())

    # enable() starts each camera streaming images; without it getImage() returns None.
    camera_front = robot.getDevice('camera')
    camera_front.enable(timestep)

    camera_left = robot.getDevice('camera_left')
    camera_left.enable(timestep)

    camera_right = robot.getDevice('camera_right')
    camera_right.enable(timestep)

    # Setting position to infinity puts the motor in velocity-control mode
    # (drive forever) instead of position-control mode (move to an angle then stop).
    left_motor = robot.getDevice("left wheel motor")
    left_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)

    right_motor = robot.getDevice("right wheel motor")
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    print('Camera resolution front:', camera_front.getWidth(), 'x', camera_front.getHeight())
    print('Camera resolution left:', camera_left.getWidth(), 'x', camera_left.getHeight())
    print('Camera resolution right:', camera_right.getWidth(), 'x', camera_right.getHeight())

    logger = TrialLogger()

    target_node = robot.getFromDef("TARGET_BALL")
    target_pos = target_node.getPosition()
    self_node = robot.getSelf()

    # Enable the proximity sensors only for eval (collision detection), not navigation.
    ps_sensors = [robot.getDevice(f"ps{i}") for i in range(8)]
    for sensor in ps_sensors:
        sensor.enable(timestep)

    wall_detector = WallDetector()

    # Main control loop.
    while robot.step(timestep) != -1:
        dist_to_target = math.dist(self_node.getPosition()[:2], target_pos[:2])
        collided = any(s.getValue() > COLLISION_THRESHOLD for s in ps_sensors)
        timed_out = robot.getTime() > MAX_TRIAL_SECONDS

        outcome = "success" if dist_to_target < TARGET_RADIUS else \
                  "collided" if collided else \
                  "timed_out" if timed_out else None

        img_bgr_front = get_camera_bgr(camera_front)
        # camera_left is enabled but not currently used for navigation logic below.
        img_bgr_left = get_camera_bgr(camera_left)
        img_bgr_right = get_camera_bgr(camera_right)

        # Front camera drives wall-ahead/wall-left detection; right camera
        # drives wall-right detection.
        wall_status = wall_detector.update(img_bgr_front)
        wall_status_right = detect_walls_status_right(img_bgr_right, split_ratio=RIGHT_CAMERA_SPLIT_RATIO)

        print("vision-> wall_ahead:", wall_status['wall_ahead'],
              "wall_left:", wall_status['wall_left'],
              "wall_front_right:", wall_status['wall_front_right'],
              "| densities L/C/R:", round(wall_status['left_density'], 4),
              round(wall_status['center_density'], 4),
              round(wall_status['right_density'], 4))

        # Check whether the red target is in view and which way it's offset.
        target_visible, target_direction, mask = detect_target(img_bgr_front)

        left_velocity, right_velocity = decide_velocities(
            wall_status, wall_status_right, target_visible, target_direction
        )
        left_motor.setVelocity(left_velocity)
        right_motor.setVelocity(right_velocity)

        logger.log_row([
            round(robot.getTime(), 3),
            wall_status['wall_ahead'], wall_status['wall_left'], wall_status['wall_front_right'],
            wall_status_right['wall_right'],
            round(wall_status['left_density'], 5), round(wall_status['center_density'], 5),
            round(wall_status['right_density'], 5), round(wall_status_right['mean_right'], 3),
            target_visible, target_direction,
            left_velocity, right_velocity,
            round(dist_to_target, 4), collided, outcome or '',
        ])

        # Visualize the Canny edge map used for the wall density calculation.
        cv2.imshow("Edges", wall_status['edges'])

        if target_visible:
            print('Target detected! Direction:', target_direction)
            # Visualize exactly which pixels were classified as "red target".
            cv2.imshow("Target Mask", mask)

        # Show a resized preview of what the robot's front and right cameras currently see.
        cv2.imshow("Robot Camera Feed", cv2.resize(img_bgr_front, (300, 300)))
        cv2.imshow("Right Camera", cv2.resize(wall_status_right['img_bottom_right'], (300, 300)))

        # Required for OpenCV to actually paint/refresh the windows each frame.
        cv2.waitKey(1)

        print("vision-right-> wall_right:", wall_status_right['wall_right'],
              "mean:", round(wall_status_right['mean_right'], 4))

        if outcome:
            robot.simulationQuit(0)
            break

    # Close all preview windows once the control loop ends.
    cv2.destroyAllWindows()
    logger.close()


if __name__ == '__main__':
    main()
