"""Turns wall detection results into wheel velocities."""

from config import WHEEL_SPEED, WALL_FOLLOW_CURVE_FACTOR


def decide_velocities(wall_status, wall_status_right):
    """Right-hand wall-following: always try to keep a wall on the right,
    breaking that only to avoid a wall closing in ahead or from either side.

    Uses all three front-camera zone signals (wall_ahead/wall_left/
    wall_front_right), not just wall_ahead -- an earlier version dropped
    wall_left/wall_front_right too, on the theory that only target-direction
    steering was responsible for the concave-pocket looping bug this replaced.
    That theory was wrong: those two are the front camera's own left-vs-right
    zone comparison, which is what catches a wall closing in from an angle
    before it's dead-center. Without them the robot drove straight through a
    2+ second approach with a wall clearly visible and growing in one zone
    (left_density 0.55->0.85, or the front camera's right_density 0.57->0.74)
    while wall_ahead (center-only) and the side camera's wall_right both
    stayed False the whole time -- see run_log_20260904_222807.csv and
    run_log_20260904_222835.csv. wall_left/wall_front_right are plain
    collision-relevant geometry, not target-seeking, so restoring them here
    doesn't reintroduce the looping bug -- only target_visible/
    target_direction (still computed and logged in maze_nav.py for eval, just
    not read here) caused that, by letting "steer toward the target"
    interrupt wall avoidance mid-escape.
    """
    if wall_status['wall_ahead']:
        if wall_status_right['wall_right']:
            # Boxed in on the right too -> turn left, away from both.
            return 0.0, WHEEL_SPEED
        # Right is open -> turn right, toward the wall we want to hug.
        return WHEEL_SPEED, 0.0

    if wall_status['wall_left']:
        # Wall closing in from the left -> steer right, away from it.
        return WHEEL_SPEED, 0.0

    if wall_status['wall_front_right']:
        # Wall closing in from the right-front -> steer left, away from it.
        return 0.0, WHEEL_SPEED

    if not wall_status_right['wall_right']:
        # Nothing detected anywhere -> curve right to find a wall to hug,
        # rather than driving straight (which would drift away from it).
        return WHEEL_SPEED, WHEEL_SPEED * WALL_FOLLOW_CURVE_FACTOR

    # Wall on the right, nothing closing in -> hug it, straight.
    return WHEEL_SPEED, WHEEL_SPEED
