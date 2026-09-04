"""Turns wall/target detection results into wheel velocities."""

from config import WHEEL_SPEED


def decide_velocities(wall_status, wall_status_right, target_visible, target_direction):
    """Priority: avoid walls first, then steer toward the target, else drive straight.

    A simple right-hand-follow-ish scheme: when blocked ahead, turn toward
    whichever side is open.
    """
    if wall_status['wall_ahead']:
        if wall_status_right['wall_right']:
            # Right side also blocked -> turn left.
            return 0.0, WHEEL_SPEED
        # Right side open -> turn right.
        return WHEEL_SPEED, 0.0

    if wall_status['wall_left']:
        # Wall hugging the left -> steer away from it (turn right).
        return WHEEL_SPEED, 0.0

    if wall_status['wall_front_right']:
        # Wall hugging the right -> steer away from it (turn left).
        return 0.0, WHEEL_SPEED

    if target_visible and target_direction == 'left':
        return 0.0, WHEEL_SPEED

    if target_visible and target_direction == 'right':
        return WHEEL_SPEED, 0.0

    # target_direction == 'center' (or no target) -> drive straight.
    return WHEEL_SPEED, WHEEL_SPEED
