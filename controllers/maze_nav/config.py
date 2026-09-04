"""Tunable constants for the maze_nav controller."""

BLUR_ENABLED = False
BLUR_SIZE = 9

# Trial-termination constants (supervisor/sensor-driven, eval-only -- never
# fed into the navigation logic). Starting values, not yet empirically tuned
# against a real run:
#   - TARGET_RADIUS: meters from the target center that counts as "reached".
#     0.08 is a bit over half a 0.15m grid cell.
#   - COLLISION_THRESHOLD: e-puck ps0-ps7 raw units past which a wall/obstacle
#     is considered touching. This proto ships no documented lookup table --
#     watch the printed ps values on a real run near a wall and adjust.
#   - MAX_TRIAL_SECONDS: sim-time timeout per trial, backstopping a robot
#     that gets stuck without colliding or reaching the target.
TARGET_RADIUS = 0.08
COLLISION_THRESHOLD = 80
MAX_TRIAL_SECONDS = 120.0

# Wall detection (front camera, edge-density based).
# Empirical cutoff: below this edge density, a zone is considered "wall very
# close" (close enough that it fills the zone as a flat, low-texture surface
# with few edges). See vision.WallDetector.
WALL_DENSITY_EPSILON = 0.005
# A wall must be seen for this many consecutive frames before it's reported,
# which debounces single-frame noise.
WALL_CONSECUTIVE_FRAMES = 3

# Wall detection (right camera, brightness based).
RIGHT_CAMERA_SPLIT_RATIO = 0.4
# Known limitation: this is the one detector keyed on ABSOLUTE intensity, and
# it is the one that fails when the world is dimmed -- both the wall and
# no-wall means drop below the fixed cutoff, so everything reads as "wall".
# See RESULTS.md, Problem 6; the fix is to key on the brightness CHANGE
# rather than its level.
RIGHT_WALL_BRIGHTNESS_THRESHOLD = 195.0

# Navigation.
WHEEL_SPEED = 3.0
