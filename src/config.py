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
LIFT_ENABLED = True


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
LINE_MAX_WIDTH_PX = 800

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
# All markers are coloured squares. Each spot has a unique colour.
#   HOME  = green  square
#   EXIT  = purple square
#   PS1   = grey   square  (Car 1)
#   PS2   = red    square  (Car 2)
# -----------------------------------------------------------------------------

# Minimum contour area to consider a marker valid (filters noise)
MIN_MARKER_AREA = 3000

# HOME — green square  (H=78 S=135 V=185)
HOME_HSV_LOW  = [50, 35, 60]
HOME_HSV_HIGH = [95, 180, 220]

# EXIT — purple square  (H=129 S=125 V=152, can be dark so V min lowered)
EXIT_HSV_LOW  = [120,  60,  40]
EXIT_HSV_HIGH = [145, 255, 255]

# PS1 (Car 1) — dark navy blue square (H≈115-125, high S, low V)
PS1_HSV_LOW  = [100, 100, 20]
PS1_HSV_HIGH = [125, 255, 160]

# PS2 (Car 2) — red square  (H=6 S=207 V=244, wraps around in HSV → two ranges)
PS2_HSV_LOW_1  = [0,  150,  100]
PS2_HSV_HIGH_1 = [12, 255,  255]
PS2_HSV_LOW_2  = [168, 150, 100]
PS2_HSV_HIGH_2 = [179, 255, 255]

# Keep MARKER_HSV_LOW/HIGH pointing at green for tune_camera.py compatibility
MARKER_HSV_LOW  = HOME_HSV_LOW
MARKER_HSV_HIGH = HOME_HSV_HIGH


# -----------------------------------------------------------------------------
# 4. NAVIGATION / PID
# -----------------------------------------------------------------------------

# Shape-seeking steering PID
# Error unit: pixels of lateral offset from frame center
STEER_KP = 0.0015  # Proportional – increase if slow to center
STEER_KI = 0.0     # Integral     – increase if steady-state drift
STEER_KD = 0.001   # Derivative   – increase if oscillating

STEER_I_MAX = 0.3

# Dead zone: ignore x_error smaller than this (pixels) — drives straight when centered
STEER_DEAD_ZONE_PX = 25

# PID loop tick rate
PID_DT = 0.033  # ~30 Hz

# Shape seek parameters
SHAPE_MIN_AREA   = 3000   # minimum px² to consider a detection valid
SHAPE_CLOSE_AREA      = 180000  # px² — stop when approaching EXIT/HOME
SHAPE_CLOSE_AREA_SPOT = 80000   # px² — stop when approaching parking spot (farther away)
SHAPE_CENTERED_PX = 80    # x_error below this = shape is centered enough to drive straight


# -----------------------------------------------------------------------------
# 5. MOTOR SPEEDS
# -----------------------------------------------------------------------------

# Normal line-following forward speed
MOTOR_BASE_SPEED = 0.28

# Speed when carrying a car — slower keeps the final parking approach controlled
MOTOR_CARRY_SPEED = 0.36

# Max speed either motor can reach
MOTOR_MAX_SPEED = 0.55

# Dead-band: speeds below this are set to 0
MOTOR_MIN_SPEED = 0.18

# Slow speed used when approaching a stopping marker
MOTOR_CREEP_SPEED = 0.22

# Speed used when spinning in place to search for line
MOTOR_SEARCH_SPIN_SPEED = 0.22

# Brief settle pause when transitioning states (seconds)
MOTOR_SETTLE_TIME_S = 0.15


# -----------------------------------------------------------------------------
# 6. LIFT SERVO
# -----------------------------------------------------------------------------

# BCM GPIO pins for lift servos (two FS90 servos moving in sync)
SERVO_GPIO_PIN   = 12   # BCM GPIO 12 — Physical Pin 32
SERVO_GPIO_PIN_2 = 13   # BCM GPIO 13 — Physical Pin 33

# Pulse widths in microseconds (continuous rotation servos)
# These are spin direction pulses — servo runs for SERVO_TRAVEL_TIME_S then stops
SERVO_PULSE_UP_US   = 1750   # Spin direction that raises the platform
SERVO_PULSE_DOWN_US = 1290   # Spin direction that lowers the platform

# How long to spin before cutting signal (seconds)
SERVO_TRAVEL_TIME_S = 0.5

# How long to reverse straight back after lifting (seconds)
LIFT_BACKUP_TIME_S = 2.4
# Speed during backup — slower = smoother and straighter
LIFT_BACKUP_SPEED  = 0.28
# How long to reverse after dropping the car at PS1/PS2
DROP_OFF_BACKUP_TIME_S = 3.6
# Speed while backing away from the parked car
DROP_OFF_BACKUP_SPEED  = 0.25
# How long to turn right after backup (aligns robot parallel to wall)
LIFT_TURN_TIME_S        = 0.75
# Maximum time to keep turning right while looking for the parking spot marker
LIFT_TURN_SEARCH_TIMEOUT_S = 3.0
# Leg 1 of L: drive straight forward along the wall toward PS1
DELIVER_FORWARD_TIME_S  = 0.9
# Corner of L: turn left to face PS1 directly
DELIVER_TURN_TIME_S     = 2.8


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
DEBUG_SHOW_PREVIEW = False

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
