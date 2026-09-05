"""Tunable constants for the maze_nav controller."""

BLUR_ENABLED = False
BLUR_SIZE = 9

# Trial-termination constants (supervisor/sensor-driven, eval-only -- never
# fed into the navigation logic). Starting values, not yet empirically tuned
# against a real run:
#   - TARGET_RADIUS: meters from the target center that counts as "reached".
#     Raised 0.08 -> 0.1 after a real run (run_log_20260904_150723.csv) showed
#     the target Ball's own physical radius trips the ps-sensor collision
#     check (dist_to_target=0.0823, just above the old 0.08) one frame before
#     dist_to_target would have crossed the threshold -- the robot had
#     genuinely reached the target but the trial logged "collided" instead
#     of "success". 0.1 clears that race with real margin (the same run's
#     dist_to_target was already 0.0881-0.0919 several frames earlier).
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
TARGET_RADIUS = 0.1
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
# Once any of wall_ahead/wall_left/wall_front_right triggers, hold it True for
# this many extra frames regardless of the next reading. Without this, a
# single-frame turn slightly reduces the density that triggered it (the wall
# looks a bit smaller after turning even 32ms worth), which immediately reset
# the debounce counter and snapped the robot straight back to full-speed-
# straight one frame before it hit the wall (see run_log_20260904_222338.csv /
# _222357.csv: wall_ahead=True for exactly one row, then False, then collided
# a frame or two later). This is a fixed TIME (frames), but the turn rate
# during a pivot (one wheel at 0, the other at WHEEL_SPEED) scales directly
# with WHEEL_SPEED -- so raising WHEEL_SPEED without rescaling this would turn
# further per hold (e.g. 3.0->5.0 would turn ~32deg -> ~54deg in the same 15
# frames), risking overturning into the opposite wall. Rescaled 15 -> 9
# (x 3.0/5.0) to hold the same ~32deg turn angle this was originally tuned
# against; re-derive again if WHEEL_SPEED changes further.
WALL_TURN_HOLD_FRAMES = 9

# Wall detection (right camera, brightness based).
RIGHT_CAMERA_SPLIT_RATIO = 0.4
# Known limitation: this is the one detector keyed on ABSOLUTE intensity, and
# it is the one that fails when the world is dimmed -- both the wall and
# no-wall means drop below the fixed cutoff, so everything reads as "wall".
# See RESULTS.md, Problem 6; the fix is to key on the brightness CHANGE
# rather than its level.
RIGHT_WALL_BRIGHTNESS_THRESHOLD = 195.0

# Navigation.
# Raised 3.0 -> 5.0 rad/s (linear ~0.0615 -> ~0.1025 m/s at the e-puck's
# ~0.0205m wheel radius) -- still comfortably under the real e-puck's ~6.28
# rad/s hardware ceiling, so this stays realistic for eventual sim-to-real
# transfer. WALL_TURN_HOLD_FRAMES below was rescaled to match -- see its
# comment; turning radius (WALL_FOLLOW_CURVE_FACTOR) and MAX_TRIAL_SECONDS
# don't need adjustment for a speed change (see chat, geometry/threshold
# derivations respectively).
WHEEL_SPEED = 5.0
# Right-hand wall-following (see navigation.decide_velocities): when no wall
# is detected on the right, curve right at this fraction of WHEEL_SPEED on
# the right wheel (rather than driving straight, which would drift away from
# the wall being hugged, or pivoting in place, which loses forward progress).
# 0.4 produced a turning radius of ~0.06m (R = (axle/2)*(vL+vR)/(vL-vR), using
# the e-puck's ~0.0205m wheel radius and ~0.052m axle) -- far smaller than the
# ~0.225m corridor half-width at the current 0.5m cell size, so a robot that
# started mid-corridor just circled forever without ever reaching a wall (see
# run_log_20260904_222129.csv: wall_ahead/wall_right False for all 754 rows).
# 0.85 -> R~=0.32m, comfortably past the corridor half-width so the search
# curve actually sweeps into a wall; still an estimate from the geometry
# above, not yet confirmed against a real run.
WALL_FOLLOW_CURVE_FACTOR = 0.85

# Spawn randomization (see spawn.py). Resting heights match the E-puck/Ball
# proto geometry -- constant across every world regardless of grid/cell size
# (see tools/generate_maze_world.py's module docstring for the derivation).
EPUCK_RESTING_Z = 0.0248092
BALL_RESTING_Z = 0.030379
# Reject a random start/target pair closer than this fraction of the grid
# size (in cells) apart, so trials aren't trivially short.
SPAWN_MIN_SEPARATION_FRACTION = 0.5
