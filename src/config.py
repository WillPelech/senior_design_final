# =============================================================================
# config.py  –  ALL TUNABLE PARAMETERS FOR THE AI VALET ROBOT
# =============================================================================
# This is the single source of truth for every number you might want to adjust
# during testing and tuning. Nothing is hard-coded anywhere else.
#
# Sections:
#   1. Camera
#   2. ArUco Detection
#   3. Navigation / PID
#   4. State Machine Thresholds
#   5. Motor Speeds
#   6. Forklift Servo
#   7. Search Behavior
#   8. Safety
#   9. Debug / Logging
# =============================================================================


# -----------------------------------------------------------------------------
# 1. CAMERA
# -----------------------------------------------------------------------------

# Resolution fed to the detection pipeline. Lower = faster FPS, less accurate pose.
# 640x480 is a good balance on Pi 4B (~30 FPS).
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480

# Target capture framerate (picamera2 will try to match this)
CAMERA_FPS = 30

# USB webcam device index (used when picamera2 is unavailable). 0 = first USB cam.
USB_CAMERA_INDEX = 0

# Camera horizontal field of view in degrees (Pi Camera Module 3 wide = 102°,
# standard module 2 = 62°). Used only for reference / future FOV math.
CAMERA_FOV_H_DEG = 62.0

# Intrinsic calibration matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
# These are approximate defaults for Pi Camera Module 2 at 640x480.
# Replace with values from a proper checkerboard calibration for best accuracy.
CAMERA_MATRIX = [
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
]

# Distortion coefficients [k1, k2, p1, p2, k3]
# Run camera_calibration.py with a checkerboard to get real values.
CAMERA_DIST_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]


# -----------------------------------------------------------------------------
# 2. ARUCO DETECTION
# -----------------------------------------------------------------------------

# ArUco dictionary to use. 4X4_50 is fast and reliable for short-range work.
# Options: cv2.aruco.DICT_4X4_50 / DICT_5X5_100 / DICT_6X6_250 etc.
ARUCO_DICT_NAME = "DICT_4X4_50"

# ID of the marker attached to the toy car.
# Print this marker and tape it to the car roof.
ARUCO_CAR_MARKER_ID = 0

# ID of the marker at the home/return position (set to None to disable return).
ARUCO_HOME_MARKER_ID = 1

# Physical side length of the printed ArUco marker in METERS.
# Measure the black border edge-to-edge after printing.
# Larger marker = detectable from farther away.
ARUCO_MARKER_SIZE_M = 0.05  # 5 cm

# Minimum ArUco corner detection confidence (perimeter filter).
# Markers whose detected perimeter is smaller than this fraction of the max
# are discarded. Range 0.0–1.0.
ARUCO_MIN_PERIMETER_RATE = 0.03

# How many consecutive frames the marker must be missing before the robot
# considers it truly lost and transitions to SEARCHING.
ARUCO_LOST_FRAME_THRESHOLD = 15


# -----------------------------------------------------------------------------
# 3. NAVIGATION / PID
# -----------------------------------------------------------------------------

# --- Steering PID (controls left/right differential based on tvec[0]) ---
# Error unit: meters of lateral offset from camera center-line.

STEER_KP = 1.8    # Proportional gain  – increase if robot is slow to center
STEER_KI = 0.05   # Integral gain      – increase if robot has steady-state offset
STEER_KD = 0.3    # Derivative gain    – increase if robot oscillates/overshoots

# Max integral accumulator (anti-windup) in motor-speed units
STEER_I_MAX = 0.4

# --- Approach PID (controls forward speed based on tvec[2] distance) ---
# Error unit: meters (target_distance - current_distance).

APPROACH_KP = 1.2
APPROACH_KI = 0.02
APPROACH_KD = 0.15
APPROACH_I_MAX = 0.3

# PID loop tick rate in seconds (should match main loop sleep)
PID_DT = 0.033  # ~30 Hz


# -----------------------------------------------------------------------------
# 4. STATE MACHINE THRESHOLDS
# -----------------------------------------------------------------------------

# ALIGNING: x-error (meters) below which we consider the robot "centered"
# and ready to start approaching.
ALIGN_X_THRESHOLD_M = 0.015  # 1.5 cm lateral error = good enough

# APPROACHING: distance (tvec[2], meters) at which we stop driving and
# transition to LIFTING. This should be just far enough for forks to be under.
DOCK_DISTANCE_M = 0.12  # 12 cm from camera to marker

# APPROACHING: if tvec[2] > this, the robot is too far to have good steering
# control so it drives forward fast and steers gently.
FAR_DISTANCE_M = 0.60  # 60 cm

# RETURNING: distance from home marker at which we consider ourselves "home"
HOME_DISTANCE_M = 0.15  # 15 cm


# -----------------------------------------------------------------------------
# 5. MOTOR SPEEDS
# -----------------------------------------------------------------------------
# All speeds are in range 0.0 – 1.0 (fraction of max duty cycle sent to HAT).
# Adafruit MotorKit uses -1.0 to +1.0 (negative = reverse).

# Base forward speed during approach phase
MOTOR_BASE_SPEED = 0.45

# Speed cap: steering PID can never command more than this to either motor
MOTOR_MAX_SPEED = 0.75

# Minimum speed below which a motor is just held at 0 (dead-band)
MOTOR_MIN_SPEED = 0.12

# Slow forward creep speed used right before docking (close approach)
MOTOR_CREEP_SPEED = 0.20

# Speed used when spinning in place during SEARCHING
MOTOR_SEARCH_SPIN_SPEED = 0.30

# Speed used when returning home
MOTOR_RETURN_SPEED = 0.40

# Brief full-stop settle time (seconds) when transitioning states
MOTOR_SETTLE_TIME_S = 0.15


# -----------------------------------------------------------------------------
# 6. GRIPPER SERVO  (Thingiverse thing:2415 – Mini Servo Gripper by jjshortcut)
# -----------------------------------------------------------------------------
# The gripper is a parallel-jaw design driven by a single SG90 / MG90S servo.
# The servo horn drives a rack-and-pinion linkage that opens/closes both jaws
# symmetrically. Mount the gripper at the FRONT of the chassis on a riser
# so the jaw centre-line is at toy-car body height (~18-22mm off ground).
#
# Pulse widths must be measured empirically for your printed gripper because
# print tolerances affect where the open/closed endpoints actually land.
# Use the servo test script (tools/servo_test.py) to find your values.

# BCM GPIO pin the servo signal wire is connected to.
# If using Motor HAT servo channel, set SERVO_USE_HAT_CHANNEL = True instead.
SERVO_GPIO_PIN = 18
SERVO_USE_HAT_CHANNEL = False   # Set True if wired to HAT, False for raw GPIO
SERVO_HAT_CHANNEL = 0           # Which HAT servo channel (0 or 1), if above = True

# PWM frequency for software servo control (Hz). Standard servos = 50 Hz.
SERVO_PWM_FREQ_HZ = 50

# Pulse widths in microseconds for gripper jaw positions.
# Tune these first before anything else – wrong values will strip the servo gears.
SERVO_PULSE_OPEN_US   = 1000   # Jaws fully open  (wider than toy car body)
SERVO_PULSE_CLOSED_US = 1600   # Jaws fully closed (gripping toy car body)
SERVO_PULSE_HALF_US   = 1300   # Jaws half-open (safe start position / calibration)

# Maximum safe pulse – do NOT go above this or the gripper mechanism will bind.
# Find this by slowly increasing from SERVO_PULSE_OPEN_US until jaws just touch.
SERVO_PULSE_MAX_SAFE_US = 1700

# Grip force tuning: close to this pulse instead of CLOSED if grip is too tight
# and crushing/deforming the toy car. Reduce to grip lighter.
SERVO_PULSE_GRIP_US = 1550   # Default grip pulse (between half and closed)

# Time (seconds) to wait after commanding servo before moving motors again.
# Give the servo time to physically complete its travel.
SERVO_TRAVEL_TIME_S = 0.6

# How wide the gripper jaws open in mm at SERVO_PULSE_OPEN_US.
# Measure this on your printed gripper. Used only for documentation / comments.
GRIPPER_JAW_OPEN_MM   = 55.0  # mm – must be wider than your toy car body width
GRIPPER_JAW_CLOSED_MM = 28.0  # mm – approximate grip width at SERVO_PULSE_GRIP_US

# Hot Wheels / Matchbox car body width is typically 28-32mm.
# Make sure GRIPPER_JAW_OPEN_MM > car width + ~10mm of clearance.

# Dock distance adjustment: gripper approach stops slightly further away than
# a forklift would, so the jaws can close around the car body cleanly.
# This value is added to DOCK_DISTANCE_M at runtime.
GRIPPER_STANDOFF_EXTRA_M = 0.02  # 2cm extra clearance for jaw closure

# Whether to open gripper automatically when returning home after a successful grip
GRIPPER_AUTO_RELEASE_ON_RETURN = True


# -----------------------------------------------------------------------------
# 7. SEARCH BEHAVIOR
# -----------------------------------------------------------------------------

# Maximum number of full spin cycles before giving up and stopping.
# One cycle = robot spins 360°.
SEARCH_MAX_CYCLES = 3

# Degrees to spin per search step before checking for marker again.
# Smaller = more frequent checks but slower full rotation.
SEARCH_STEP_DEG = 15.0

# Time (seconds) the robot pauses between each search spin step.
SEARCH_STEP_PAUSE_S = 0.10

# After losing the marker mid-approach, the robot backs up slightly before
# re-entering SEARCHING. Distance in seconds of reverse drive.
SEARCH_BACKUP_TIME_S = 0.3
SEARCH_BACKUP_SPEED  = 0.25


# -----------------------------------------------------------------------------
# 8. SAFETY
# -----------------------------------------------------------------------------

# If no frame is received from the camera for this many seconds, emergency stop.
SAFETY_CAMERA_TIMEOUT_S = 2.0

# If the robot has been in APPROACHING for longer than this (seconds) without
# reaching dock distance, assume something is wrong and stop.
SAFETY_APPROACH_TIMEOUT_S = 20.0

# If the robot has been in SEARCHING for longer than this (seconds), stop.
SAFETY_SEARCH_TIMEOUT_S = 30.0

# Emergency stop: if tvec[2] ever drops below this while motors are running,
# hard-stop immediately (obstacle too close, or overshot).
SAFETY_MIN_DISTANCE_M = 0.06  # 6 cm


# -----------------------------------------------------------------------------
# 9. DEBUG / LOGGING
# -----------------------------------------------------------------------------

# Show live OpenCV preview window with ArUco overlay (requires display / VNC).
# Set False on headless Pi deployments.
DEBUG_SHOW_PREVIEW = True

# Draw pose axes on detected marker in the preview window.
DEBUG_DRAW_AXES = True

# Print PID values, tvec, and motor speeds to stdout each loop tick.
DEBUG_PRINT_TELEMETRY = True

# Telemetry print rate (every N ticks). 1 = every tick, 10 = every 10th tick.
DEBUG_TELEMETRY_EVERY_N = 5

# Save annotated frames to disk for post-run review.
DEBUG_SAVE_FRAMES = False
DEBUG_SAVE_DIR = "/tmp/valet_frames"
DEBUG_SAVE_EVERY_N = 10  # Save every Nth frame to avoid filling disk

# Log level: "DEBUG", "INFO", "WARNING", "ERROR"
LOG_LEVEL = "INFO"
