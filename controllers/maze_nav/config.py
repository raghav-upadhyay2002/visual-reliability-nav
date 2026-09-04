"""Tunable constants for the maze_nav controller."""

BLUR_ENABLED = False
BLUR_SIZE = 9

# Trial-termination constants (supervisor/sensor-driven, eval-only -- never
# fed into the navigation logic). Starting values, not yet empirically tuned
# against a real run:
#   - TARGET_RADIUS: meters from the target center that counts as "reached".
#     0.08 is a bit over half of what was a 0.15m grid cell; cells are now
#     0.5m (see tools/generate_maze_world.py DEFAULT_CELL) so this is a
#     smaller fraction of a cell than originally tuned, but still a
#     reasonable "reached" radius -- not yet re-validated against a real run
#     at the new scale.
#   - COLLISION_THRESHOLD: e-puck ps0-ps7 raw units past which a wall/obstacle
#     is considered touching. This proto ships no documented lookup table.
#     80 was a pure guess and turned out to sit INSIDE the sensor noise floor
#     -- observed open-space readings in a real run ranged ~56-76 with noise
#     spikes to 80+, so it fired constantly with nothing nearby (see the
#     printed ps-values diagnostic in maze_nav.py's main loop). 250 is a
#     wide-margin guess above that noise ceiling, not yet confirmed against
#     an actual wall touch -- still needs one real-contact sample to pin down
#     precisely; watch the printed ps values again once the robot can
#     actually reach a wall on purpose.
#   - MAX_TRIAL_SECONDS: sim-time timeout per trial, backstopping a robot
#     that gets stuck without colliding or reaching the target. Raised from
#     120s -> 300s alongside the 0.15m -> 0.5m cell-size increase (~3.3x more
#     physical distance to cover at the same WHEEL_SPEED); not yet confirmed
#     this is enough headroom for the longest paths on the biggest (10x10)
#     mazes.
TARGET_RADIUS = 0.08
COLLISION_THRESHOLD = 250
MAX_TRIAL_SECONDS = 300.0

# Wall detection (front camera, edge-density based).
# Empirical cutoff: below this edge density, a zone is considered "wall very
# close" (close enough that it fills the zone as a flat, low-texture surface
# with few edges). See vision.WallDetector.
WALL_DENSITY_EPSILON = 0.005
# A wall must be seen for this many consecutive frames before it's reported,
# which debounces single-frame noise.
WALL_CONSECUTIVE_FRAMES = 3
# Separate, shorter debounce for the straight-ahead check specifically (see
# vision_color.ColorWallDetector). Every collision in run_log_*.csv showed
# center_density already unambiguous (0.75-1.0) for many frames before impact,
# but the robot was still closing at WHEEL_SPEED with no margin to spare --
# react on 2 confirming frames instead of 3.
WALL_AHEAD_CONSECUTIVE_FRAMES = 2

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

# Spawn randomization (see spawn.py). Resting heights match the E-puck/Ball
# proto geometry -- constant across every world regardless of grid/cell size
# (see tools/generate_maze_world.py's module docstring for the derivation).
EPUCK_RESTING_Z = 0.0248092
BALL_RESTING_Z = 0.030379
# Reject a random start/target pair closer than this fraction of the grid
# size (in cells) apart, so trials aren't trivially short.
SPAWN_MIN_SEPARATION_FRACTION = 0.5
