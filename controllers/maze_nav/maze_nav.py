from controller import Robot
import numpy as np
import cv2
import os
import datetime 
import csv

BLUR_ENABLED= False

BLUR_SIZE=9

def blur_cam(img_bgr):
    if not BLUR_ENABLED:
        return img_bgr

    return cv2.GaussianBlur(img_bgr, (BLUR_SIZE, BLUR_SIZE),0)


def detect_target(img_bgr):
    """Look for the red target in a BGR camera frame.

    Returns (found, direction, mask) where direction is 'left'/'center'/'right'
    (None if not found), and mask is the binary image of pixels classified as red.
    """
    # HSV separates color (hue) from lighting (value), which makes
    # thresholding a specific color far more reliable than in raw BGR.
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # Red wraps around the hue wheel (near 0 and near 180), so two ranges
    # are needed to catch both ends and cover the full red band.
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([7, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # inRange() gives a binary mask of pixels inside each range; combining
    # them covers both halves of the red hue band.
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Number of red pixels found; used as a simple "is the target visible" threshold.
    pixel_count = cv2.countNonZero(mask)

    if pixel_count > 50:
        # Average x-position of every red pixel gives the target's horizontal
        # center in the frame, which tells us which way to steer.
        ys, xs = np.where(mask > 0)
        cx = np.mean(xs)
        width = img_bgr.shape[1]

        if cx < width * 0.4:
            direction = 'left'
        elif cx > width * 0.6:
            direction = 'right'
        else:
            direction = 'center'

        return True, direction, mask
    else:
        return False, None, mask


def detect_edges(img_bgr):
    """Grayscale + blur + Canny -> a binary edge map of the frame."""
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # Smooth out noise first so Canny doesn't pick up false edges from grain/texture.
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # Canny returns a binary image: white pixels mark detected edges.
    edges = cv2.Canny(blurred, 50, 150)
    return edges


'''def detect_lines(edges):
    """Turn edge pixels into line segments via the probabilistic Hough transform.

    Computed for potential future use (e.g. recognizing straight maze walls
    more precisely) but not currently consumed by get_wall_status_vision,
    which relies on edge density instead.
    """
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180,
        threshold=20,
        minLineLength=15,
        maxLineGap=5
    )
    return lines'''


def get_wall_status_vision(img_bgr):
    """Estimate wall presence ahead/left/right from the front camera's edge density.

    Uses module-level counters so a wall must be seen for several consecutive
    frames before it's reported, which debounces single-frame noise.
    """
    global center_density_count
    global left_density_count
    global right_density_count

    edges = detect_edges(img_bgr)
    #lines = detect_lines(edges)
    height, width = edges.shape

    # Split the frame into left/center/right thirds so wall presence can be
    # judged separately in each direction the robot could turn toward.
    left_zone = edges[:, 0:int(width * 0.33)]
    center_zone = edges[:, int(width * 0.33):int(width * 0.66)]
    right_zone = edges[:, int(width * 0.66):]

    # Fraction of edge pixels in each zone. Note the signal is inverted from what
    # you might expect: a wall close enough to matter fills the zone as a flat,
    # low-texture surface and produces almost NO edges, while open space (floor,
    # sky, distant scenery) always retains contrast and produces many. See
    # RESULTS.md, Problem 1 -- background scenery makes "many edges = wall" unusable.
    left_density = cv2.countNonZero(left_zone) / left_zone.size
    center_density = cv2.countNonZero(center_zone) / center_zone.size
    right_density = cv2.countNonZero(right_zone) / right_zone.size

    # Empirical cutoff: below this edge density, a zone is considered "wall very
    # close" (close enough that it fills the zone as a flat, low-texture surface
    # with few edges).
    density_epsilon = 0.005

    # Wall directly ahead: all three zones read as flat/close.
    if left_density < density_epsilon and center_density < density_epsilon and right_density < density_epsilon:
        center_density_count += 1
    else:
        center_density_count = 0
    wall_ahead = center_density_count >= 3

    # Wall hugging the left side, with an opening to the right.
    if left_density < density_epsilon and right_density > density_epsilon:
        left_density_count += 1
    else:
        left_density_count = 0
    wall_left = left_density_count >= 3

    # Wall hugging the right side, with an opening to the left.
    if right_density < density_epsilon and left_density > density_epsilon:
        right_density_count += 1
    else:
        right_density_count = 0
    wall_front_right = right_density_count >= 3

    return {
        'wall_ahead': wall_ahead,
        'wall_left': wall_left,
        'wall_front_right': wall_front_right,
        'left_density': left_density,
        'center_density': center_density,
        'right_density': right_density,
        'edges': edges
    }


def bottom_img_right(img_bgr_right, split_ratio):
    """Crop the bottom fraction (split_ratio) of the right camera's frame.

    The right side camera is aimed along the wall the robot follows, so the
    bottom strip of its image is what's closest to (and most informative about)
    that wall.
    """
    height, width = img_bgr_right.shape[:2]
    split_y = int(height * (1 - split_ratio))
    img_bottom_right = img_bgr_right[split_y:, :]
    return img_bottom_right


def detect_walls_status_right(img_bgr_right, split_ratio):
    """Detect a wall on the right from how dark the bottom strip of that camera is.

    A nearby wall fills the frame and reduces the average brightness compared
    to open floor, so a mean-brightness threshold suffices at baseline illumination.

    Known limitation: this is the one detector keyed on ABSOLUTE intensity, and it
    is the one that fails when the world is dimmed -- both the wall and no-wall
    means drop below the fixed cutoff, so everything reads as "wall". See RESULTS.md,
    Problem 6; the fix is to key on the brightness CHANGE rather than its level.
    """
    img_bottom_right = bottom_img_right(img_bgr_right, split_ratio)
    gray_right = cv2.cvtColor(img_bottom_right, cv2.COLOR_BGR2GRAY)
    mean_right = np.mean(gray_right)

    drop_threshold = 195.0
    wall_right = mean_right < drop_threshold

    return {
        'wall_right': wall_right,
        'mean_right': mean_right,
        'img_bottom_right': img_bottom_right
    }


# Webots hands us a Robot instance representing this e-puck; all devices
# (camera, motors, sensors) are accessed through it via getDevice().
robot = Robot()

# Module-level (not local to get_wall_status_vision) so it persists across control-loop
# iterations, counting consecutive frames where each zone reads "wall very close".
center_density_count = 0
left_density_count = 0
right_density_count = 0

# Simulation step size in ms. Every device must be enabled with this value,
# and robot.step(timestep) must be called once per control loop iteration
# to advance the simulation clock.
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

log_dir = os.path.dirname(os.path.abspath(__file__))
log_filename = 'run_log_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
log_path = os.path.join(log_dir, log_filename)
log_file = open(log_path, 'w', newline='')
log_writer = csv.writer(log_file)
log_writer.writerow([
    'sim_time_s',
    'wall_ahead', 'wall_left', 'wall_front_right', 'wall_right',
    'left_density', 'center_density', 'right_density', 'mean_right',
    'target_visible', 'target_direction',
    'left_velocity', 'right_velocity'
])















while robot.step(timestep) != -1:
    # Raw camera image comes back as a flat byte buffer in BGRA order.
    image_front = camera_front.getImage()
    width_front = camera_front.getWidth()
    height_front = camera_front.getHeight()

    image_left = camera_left.getImage()
    width_left = camera_left.getWidth()
    height_left = camera_left.getHeight()

    image_right = camera_right.getImage()
    width_right = camera_right.getWidth()
    height_right = camera_right.getHeight()

    # Reshape the flat buffer into a (height, width, 4) BGRA array OpenCV can use.
    img_front = np.frombuffer(image_front, np.uint8).reshape((height_front, width_front, 4))
    img_left = np.frombuffer(image_left, np.uint8).reshape((height_left, width_left, 4))
    img_right = np.frombuffer(image_right, np.uint8).reshape((height_right, width_right, 4))

    # Drop the alpha channel so it's a standard 3-channel BGR image.
    img_bgr_front = blur_cam(cv2.cvtColor(img_front, cv2.COLOR_BGRA2BGR))
    img_bgr_left = blur_cam(cv2.cvtColor(img_left, cv2.COLOR_BGRA2BGR))
    img_bgr_right = blur_cam(cv2.cvtColor(img_right, cv2.COLOR_BGRA2BGR))

    

    # Front camera drives wall-ahead/wall-left detection; right camera drives
    # wall-right detection. camera_left is enabled but not currently used for
    # navigation logic below.
    wall_status = get_wall_status_vision(img_bgr_front)
    wall_status_right = detect_walls_status_right(img_bgr_right, split_ratio=0.4)

    print("vision-> wall_ahead:", wall_status['wall_ahead'],
          "wall_left:", wall_status['wall_left'],
          "wall_front_right:", wall_status['wall_front_right'],
          "| densities L/C/R:", round(wall_status['left_density'], 4),
          round(wall_status['center_density'], 4),
          round(wall_status['right_density'], 4))

    # Check whether the red target is in view and which way it's offset.
    target_visible, target_direction, mask = detect_target(img_bgr_front)

    # Navigation priority: avoid walls first, then steer toward the target,
    # otherwise drive straight. This is a simple right-hand-follow-ish scheme:
    # when blocked ahead, turn toward whichever side is open.
    if wall_status['wall_ahead']:
        if wall_status_right['wall_right']:
            # Right side also blocked -> turn left.
            left_velocity = 0.0
            right_velocity = 3.0
        else:
            # Right side open -> turn right.
            left_velocity = 3.0
            right_velocity = 0.0

    elif wall_status['wall_left']:
        # Wall hugging the left -> steer away from it (turn right).
        left_velocity = 3.0
        right_velocity = 0.0

    elif wall_status['wall_front_right']:
        # Wall hugging the right -> steer away from it (turn left).
        left_velocity = 0.0
        right_velocity = 3.0

    elif target_visible and target_direction == 'left':
        left_velocity = 0.0
        right_velocity = 3.0

    elif target_visible and target_direction == 'right':
        left_velocity = 3.0
        right_velocity = 0.0

    else:
        # target_direction == 'center' (or no target) -> drive straight.
        left_velocity = 3.0
        right_velocity = 3.0

    left_motor.setVelocity(left_velocity)
    right_motor.setVelocity(right_velocity)

    log_writer.writerow([
        round(robot.getTime(), 3),
        wall_status['wall_ahead'], wall_status['wall_left'], wall_status['wall_front_right'],
        wall_status_right['wall_right'],
        round(wall_status['left_density'], 5), round(wall_status['center_density'], 5),
        round(wall_status['right_density'], 5), round(wall_status_right['mean_right'], 3),
        target_visible, target_direction,
        left_velocity, right_velocity
    ])
    log_file.flush()

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

# Close all preview windows once the control loop ends.
cv2.destroyAllWindows()
log_file.close()

