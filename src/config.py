# =============================================================================
# config.py  –  ALL TUNABLE PARAMETERS FOR THE AI VALET ROBOT
# =============================================================================
# Sections:
#   0. Testing Flags
#   1. Camera
#   2. Line Detection
#   3. Stopping Marker Detection
#   4. Navigation / PID
#   5. Motor Speeds
#   6. Lift Servo
#   7. Safety
#   8. Debug / Logging
# =============================================================================


# -----------------------------------------------------------------------------
# 0. TESTING FLAGS
# -----------------------------------------------------------------------------

# Set True to skip all motor/servo/GPIO hardware init (camera-only testing)
STUB_MOTORS = False

# Set False to disable lift servos entirely (line-following only mode)
LIFT_ENABLED = False


# -----------------------------------------------------------------------------
# 1. CAMERA
# -----------------------------------------------------------------------------

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 30

# USB webcam device index. Run the "Find USB camera index" command in COMMANDS.md.
USB_CAMERA_INDEX = 0

# Force USB camera even if picamera2 is installed (Pi camera broken).
FORCE_USB_CAMERA = True


# -----------------------------------------------------------------------------
# 2. LINE DETECTION  (downward-facing camera, black tape on white floor)
# -----------------------------------------------------------------------------

# Grayscale threshold to detect black tape (0–255). Lower = darker.
# Increase if white floor is being detected as line; decrease if line is missed.
LINE_THRESHOLD = 160

# Row in the frame (pixels from top) used to sample the line position.
# Use the bottom third of the frame for fastest response.
LINE_SAMPLE_ROW = int(CAMERA_HEIGHT * 0.75)

# Minimum width of the detected line in pixels (filters out noise).
LINE_MIN_WIDTH_PX = 20

# Maximum width of the detected line in pixels (filters out large dark patches).
LINE_MAX_WIDTH_PX = 200

# HSV range for blue tape (branch lines + fork junction)
BLUE_HSV_LOW  = [100, 80, 50]
BLUE_HSV_HIGH = [130, 255, 255]

# Minimum blue pixel area to confirm fork/branch line detection
FORK_MIN_AREA_PX = 300

# Number of consecutive frames blue line must be detected before acting on it
FORK_CONFIRM_FRAMES = 5

# Minimum pixel count of blue line strip to use for steering (same HSV as fork)
BLUE_LINE_MIN_WIDTH_PX = 15


# -----------------------------------------------------------------------------
# 3. STOPPING MARKER DETECTION
# (colored electrical tape shapes at each spot, seen by downward camera)
# -----------------------------------------------------------------------------

# Minimum contour area to consider a marker valid (filters noise)
# Shapes are flush with the downward camera so they fill a large portion of frame
MIN_MARKER_AREA = 8000

# All stopping markers are GREEN shapes (circle, square, hexagon, triangle)
# Differentiated by shape, not color
MARKER_HSV_LOW  = [40,  80,  50]
MARKER_HSV_HIGH = [80,  255, 255]

# Convenience aliases (same range, kept for readability)
PS1_HSV_LOW   = MARKER_HSV_LOW    # green square
PS1_HSV_HIGH  = MARKER_HSV_HIGH
PS2_HSV_LOW   = MARKER_HSV_LOW    # green hexagon
PS2_HSV_HIGH  = MARKER_HSV_HIGH
HOME_HSV_LOW  = MARKER_HSV_LOW    # green circle
HOME_HSV_HIGH = MARKER_HSV_HIGH
EXIT_HSV_LOW  = MARKER_HSV_LOW    # green triangle
EXIT_HSV_HIGH = MARKER_HSV_HIGH

# Square aspect ratio tolerance (width/height must be within this range)
SQUARE_ASPECT_MIN = 0.6
SQUARE_ASPECT_MAX = 1.4

# Minimum circularity for home circle (0=line, 1=perfect circle)
CIRCLE_CIRCULARITY_MIN = 0.65


# -----------------------------------------------------------------------------
# 4. NAVIGATION / PID
# -----------------------------------------------------------------------------

# Line-following steering PID
# Error unit: pixels of lateral offset from frame center
STEER_KP = 0.003   # Proportional – increase if slow to center
STEER_KI = 0.0001  # Integral     – increase if steady-state drift
STEER_KD = 0.001   # Derivative   – increase if oscillating

STEER_I_MAX = 0.3

# PID loop tick rate
PID_DT = 0.033  # ~30 Hz

# x-error (pixels) below which robot is considered centered on line
LINE_CENTER_THRESHOLD_PX = 20


# -----------------------------------------------------------------------------
# 5. MOTOR SPEEDS
# -----------------------------------------------------------------------------

# Normal line-following forward speed
MOTOR_BASE_SPEED = 0.40

# Max speed either motor can reach
MOTOR_MAX_SPEED = 0.70

# Dead-band: speeds below this are set to 0
MOTOR_MIN_SPEED = 0.12

# Slow speed used when approaching a stopping marker
MOTOR_CREEP_SPEED = 0.20

# Speed used when spinning in place to search for line
MOTOR_SEARCH_SPIN_SPEED = 0.30

# Brief settle pause when transitioning states (seconds)
MOTOR_SETTLE_TIME_S = 0.15


# -----------------------------------------------------------------------------
# 6. LIFT SERVO
# -----------------------------------------------------------------------------

# BCM GPIO pins for lift servos (two FS90 servos moving in sync)
SERVO_GPIO_PIN   = 18   # Pin 12 on Pi header
SERVO_GPIO_PIN_2 = 27   # Pin 13 on Pi header
SERVO_USE_HAT_CHANNEL = False
SERVO_HAT_CHANNEL     = 0

SERVO_PWM_FREQ_HZ = 50

# Pulse widths in microseconds
SERVO_PULSE_DOWN_US = 1000   # Platform lowered (travel position)
SERVO_PULSE_UP_US   = 1800   # Platform raised  (carrying car)
SERVO_PULSE_MAX_SAFE_US = 1900

# Time to wait for servo to complete travel (seconds)
SERVO_TRAVEL_TIME_S = 0.1


# -----------------------------------------------------------------------------
# 7. SAFETY
# -----------------------------------------------------------------------------

# Camera timeout – if no frame for this long, emergency stop
SAFETY_CAMERA_TIMEOUT_S = 10.0

# Max time allowed for any navigation leg before giving up
SAFETY_NAV_TIMEOUT_S = 30.0


# -----------------------------------------------------------------------------
# 8. DEBUG / LOGGING
# -----------------------------------------------------------------------------

# Show OpenCV preview window (requires display – set False when SSH only)
DEBUG_SHOW_PREVIEW = True

# Serve live MJPEG stream at http://<pi-ip>:DEBUG_STREAM_PORT
DEBUG_STREAM_VIDEO = True
DEBUG_STREAM_PORT  = 8080

# Print line-following telemetry to terminal
DEBUG_PRINT_TELEMETRY = True
DEBUG_TELEMETRY_EVERY_N = 10

# Save annotated frames to disk
DEBUG_SAVE_FRAMES = False
DEBUG_SAVE_DIR    = "/tmp/valet_frames"
DEBUG_SAVE_EVERY_N = 10

LOG_LEVEL = "INFO"
