"""Webots controller: vision-based maze navigation with eval logging.

Reads the front/left/right cameras each step, derives wall/target detections
(vision.py), turns those into wheel velocities (navigation.py), and logs a
CSV row per step (trial_logger.py) for offline reliability analysis. Trial
termination (success/collision/timeout) is supervisor/sensor-driven and
never feeds back into the navigation decisions themselves.
"""

import math
import os

from controller import Supervisor
import cv2

from config import TARGET_RADIUS, COLLISION_THRESHOLD, MAX_TRIAL_SECONDS, RIGHT_CAMERA_SPLIT_RATIO
from camera_utils import get_camera_bgr
from vision import detect_target, detect_walls_status_right, WallDetector
from vision_color import ColorWallDetector, detect_walls_status_right_color
from navigation import decide_velocities
from trial_logger import TrialLogger
from spawn import randomize_spawn
from world_meta import read_world_meta
from corruption import apply_corruption, read_corruption_config


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

    # Corruption type/severity are fixed for the whole trial (like start/target
    # spawn) -- set via env vars so the eval driver can sweep them without
    # touching this file. Defaults to ('clean', 1), i.e. no corruption.
    corruption_type, corruption_severity = read_corruption_config()
    print('Corruption condition:', corruption_type, 'severity:', corruption_severity)

    target_node = robot.getFromDef("TARGET_BALL")
    self_node = robot.getSelf()

    # Randomize where the robot starts and where the target is, each run --
    # see spawn.py. Falls back to the world's baked-in positions on worlds
    # (e.g. mazesolving_test.wbt) that don't carry the grid_size/cell_size
    # customData this needs. MAZE_NAV_SPAWN_SEED makes a specific run
    # reproducible when set (e.g. by the eval driver); unset means a fresh
    # random spawn every launch.
    seed = os.environ.get('MAZE_NAV_SPAWN_SEED')
    spawned = randomize_spawn(self_node, target_node, seed=seed)
    if spawned is not None:
        start_xy, target_xy = spawned
    else:
        start_xy = tuple(self_node.getPosition()[:2])
        target_xy = tuple(target_node.getPosition()[:2])
    target_pos = target_xy

    # Enable the proximity sensors only for eval (collision detection), not navigation.
    ps_sensors = [robot.getDevice(f"ps{i}") for i in range(8)]
    for sensor in ps_sensors:
        sensor.enable(timestep)

    # Prefer the color-based detectors (see vision_color.py) when this world
    # carries its own wall_color in customData -- they replace the
    # edge-density/brightness detectors, which stopped working once shadows
    # and surface texture were removed from every world. Worlds without that
    # customData (e.g. mazesolving_test.wbt, a different scale/convention)
    # fall back to the old detectors rather than crash.
    world_meta = read_world_meta(self_node)
    use_color_detection = 'wall_color' in world_meta
    if use_color_detection:
        wall_detector = ColorWallDetector(world_meta['wall_color'])
        right_wall_hue = wall_detector.wall_hue
    else:
        wall_detector = WallDetector()

    # Main control loop.
    while robot.step(timestep) != -1:
        dist_to_target = math.dist(self_node.getPosition()[:2], target_pos[:2])
        ps_values = [s.getValue() for s in ps_sensors]
        collided = any(v > COLLISION_THRESHOLD for v in ps_values)
        timed_out = robot.getTime() > MAX_TRIAL_SECONDS

        # TEMP diagnostic: COLLISION_THRESHOLD=80 was an untuned guess (no
        # documented lookup table for this proto) -- print raw values every
        # step so the real "nothing nearby" baseline and "actually touching"
        # values can be read off directly instead of guessed again.
        print("ps values:", [round(v, 1) for v in ps_values], "collided:", collided)

        outcome = "success" if dist_to_target < TARGET_RADIUS else \
                  "collided" if collided else \
                  "timed_out" if timed_out else None

        # Corrupt all three frames identically before any detector sees them --
        # a real degraded sensor would degrade every camera at once, not just
        # the one feeding a particular detector. No-op when corruption_type is
        # 'clean' (the default).
        img_bgr_front = apply_corruption(get_camera_bgr(camera_front), corruption_type, corruption_severity)
        # camera_left is enabled but not currently used for navigation logic below.
        img_bgr_left = apply_corruption(get_camera_bgr(camera_left), corruption_type, corruption_severity)
        img_bgr_right = apply_corruption(get_camera_bgr(camera_right), corruption_type, corruption_severity)

        # Front camera drives wall-ahead/wall-left detection; right camera
        # drives wall-right detection.
        wall_status = wall_detector.update(img_bgr_front)
        if use_color_detection:
            wall_status_right = detect_walls_status_right_color(
                img_bgr_right, right_wall_hue, split_ratio=RIGHT_CAMERA_SPLIT_RATIO
            )
        else:
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
            round(start_xy[0], 4), round(start_xy[1], 4),
            round(target_xy[0], 4), round(target_xy[1], 4),
            corruption_type, corruption_severity,
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
            print(f"Trial ended: {outcome} (t={round(robot.getTime(), 3)}s)")
            # Stop the robot in place so it's visible where/how the trial ended.
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            # Only close the whole Webots app when the batch eval driver asks
            # for it (MAZE_NAV_AUTO_QUIT=1) -- during interactive debugging,
            # closing the app mid-trial is exactly the "starts and suddenly
            # closes" problem: just end this controller run and leave the
            # scene open so the trial's end state can actually be inspected.
            if os.environ.get('MAZE_NAV_AUTO_QUIT') == '1':
                robot.simulationQuit(0)
            break

    # Close all preview windows once the control loop ends.
    cv2.destroyAllWindows()
    logger.close()


if __name__ == '__main__':
    main()
