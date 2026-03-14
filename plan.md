# AI Valet Robot - Senior Design Plan

## Hardware Stack
| Component | Details |
|-----------|---------|
| Brain | Raspberry Pi 4B |
| Camera | Raspberry Pi Camera Module (CSI ribbon, used via picamera2) |
| Chassis | Adafruit Mini Robot Rover Kit - 2WD (#2939), 156mm x 103mm, red anodized aluminum |
| Drive | 2x DC Motors (micro-servo body, 4-6V), 1x caster wheel |
| Motor Driver | Adafruit DC & Stepper Motor HAT for Raspberry Pi (I2C, sits on GPIO header) |
| Servo / Arm | Servo motor for lift arm (connected to Pi GPIO or HAT servo channel) |
| Power | TBD - separate rails recommended (5V USB-C for Pi, 6-9V for motor HAT) |

---

## High-Level Goal
Build an autonomous robot that can:
1. Detect an ArUco marker attached to a toy car
2. Navigate toward it autonomously using pose estimation (PID steering + approach)
3. Slide forklift forks under the car and lift it via servo
4. Return to a home position (optionally guided by a second ArUco marker)

---

## Architecture Overview

```
┌─────────────────────────────────────────────┐
│              Raspberry Pi 4B                │
│                                             │
│  ┌─────────────┐    ┌────────────────────┐  │
│  │  Camera     │    │  AI Detection      │  │
│  │  (capture)  │───▶│  (YOLO / ArUco /   │  │
│  └─────────────┘    │   MobileNet)        │  │
│                     └────────┬───────────┘  │
│                              │ bounding box  │
│                              ▼               │
│                     ┌────────────────────┐  │
│                     │  Navigation Logic  │  │
│                     │  (PID / rule-based)│  │
│                     └────────┬───────────┘  │
│                              │ motor cmds    │
│                              ▼               │
│                     ┌────────────────────┐  │
│                     │  Motor Controller  │  │
│                     │  (GPIO / H-bridge) │  │
│                     └────────┬───────────┘  │
└──────────────────────────────┼──────────────┘
                               │ PWM signals
                    ┌──────────▼─────────┐
                    │   2x DC Motors     │
                    │   (L wheel, R wheel│
                    └────────────────────┘
```

---

## Robot State Machine

```
         ┌──────────┐
    ──▶  │ SEARCHING│  (spinning slowly, scanning for ArUco)
         └────┬─────┘
              │ marker detected
              ▼
         ┌──────────┐
         │ ALIGNING │  (PID on tvec[0] to center marker in frame)
         └────┬─────┘
              │ x_error < threshold
              ▼
         ┌──────────┐
         │APPROACHING│ (drive forward while tvec[2] > dock_distance)
         └────┬─────┘
              │ distance <= dock_distance
              ▼
         ┌──────────┐
         │ GRIPPING │  (stop motors, close gripper jaws around car)
         └────┬─────┘
              │ jaws closed
              ▼
         ┌──────────┐
         │ RETURNING│  (navigate back to home - optional)
         └────┬─────┘
              │ at home
              ▼
         ┌──────────┐
         │   DONE   │
         └──────────┘
```

---

## Phase Breakdown

### Phase 1 - Hardware Setup
- [ ] Assemble chassis (motors, wheels, caster)
- [ ] Choose and wire motor driver (TB6612 or Motor HAT)
- [ ] Mount Raspberry Pi 4B on top plate
- [ ] Print and assemble Thingiverse thing:2415 gripper with SG90 servo
- [ ] Mount gripper on front of chassis at car-body height (~18-22mm)
- [ ] Mount camera (front-facing, above gripper)
- [ ] Wire power (separate rails for Pi and motors)
- [ ] Basic motor spin test via GPIO
- [ ] Calibrate gripper servo pulse widths (open / grip / max-safe)

### Phase 2 - AI Detection Module
- [ ] Choose detection approach (see Decision section below)
- [ ] Set up Pi camera with `picamera2` or `OpenCV`
- [ ] Implement detection pipeline (target: 10+ FPS on Pi 4B)
- [ ] Output: bounding box (x, y, w, h) + confidence

### Phase 3 - Navigation / Movement Logic
- [ ] Define coordinate system (camera frame → motor commands)
- [ ] Implement centering: if target is left → turn left, if right → turn right
- [ ] Implement approach: if target is small → drive forward, large → stop
- [ ] PID loop on x-offset for smooth steering
- [ ] Distance estimation (bounding box size or ultrasonic sensor)

### Phase 4 - Integration & Testing
- [ ] End-to-end loop: capture → detect → navigate
- [ ] Tune PID gains
- [ ] Handle target lost (spin search behavior)
- [ ] Safety stop (object too close)

### Phase 5 - Demo Polish
- [ ] Reliable start/stop trigger (button or remote)
- [ ] Status LEDs or console logging
- [ ] Video recording of runs

---

## Detection Approach - DECIDED: ArUco Fiducial Markers

- Print an ArUco marker (e.g. 4x4_50 dictionary, ID 0) and attach to the toy car
- OpenCV's `cv2.aruco` module detects corners in <5ms per frame
- Gives full 6-DOF pose via `solvePnP` (x, y, z, roll, pitch, yaw)
- At 30 FPS on Pi Camera - very reliable under indoor lighting
- Fallback: if marker not visible, robot executes a slow spin search

### ArUco Pose Pipeline
```
Frame → cv2.aruco.detectMarkers() → corners[] → 
cv2.aruco.estimatePoseSingleMarkers() → rvec, tvec →
  tvec[0] = lateral offset (steer)
  tvec[2] = depth / distance (approach / stop)
```

---

## Motor Control Strategy

The 2WD chassis steers by differential drive:
- Turn left: right motor faster than left
- Turn right: left motor faster than right
- Forward: both motors equal speed
- Spin in place: motors opposite directions

### PID on X-offset
```
error = (target_center_x) - (frame_width / 2)
left_speed  = base_speed + Kp * error
right_speed = base_speed - Kp * error
```

---

## Decisions Log

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Detection | ArUco marker | Fast, reliable, gives full pose at 30 FPS |
| Motor driver | Adafruit Motor HAT | I2C, sits on Pi header, clean Python library |
| Camera | Pi Camera Module | CSI, lowest latency, picamera2 support |
| End action | Parallel-jaw gripper | Thingiverse thing:2415, one SG90 servo, grips car body from the sides |

## Gripper Design  –  Thingiverse thing:2415

**Print:** https://www.thingiverse.com/thing:2415 (Mini Servo Gripper by jjshortcut)
- Parallel-jaw design, one SG90 / MG90S servo drives both jaws symmetrically
- Print in PLA, 0.2mm layer height, 20% infill
- Mount to the front of the chassis top plate on a riser so jaw centre-line
  is at ~18-22mm off the ground (Hot Wheels car body height)

### Servo
- **SG90 or MG90S micro servo** – 5V, 180° range
- Powered from Pi 5V rail (Pin 2), signal on GPIO 18 (Pin 12)

### Jaw calibration (do this before first full run)
1. Run `python3 tools/servo_test.py` (interactive pulse-width sender)
2. Find `SERVO_PULSE_OPEN_US` – jaws just wide enough to drive around the car (~55mm gap)
3. Find `SERVO_PULSE_GRIP_US` – jaws press lightly on car sides (don't crush)
4. Find `SERVO_PULSE_MAX_SAFE_US` – one step before mechanism binds
5. Write all three values into `src/config.py`

### Dock distance with gripper
The gripper needs the robot to stop slightly further from the car than a
fork-lift would, so the jaws have room to close. `GRIPPER_STANDOFF_EXTRA_M`
(default 2cm) is added to `DOCK_DISTANCE_M` at runtime – tune both.

## Open Questions (still to resolve)

1. **Power** - Separate rails for Pi (5V USB-C) and motors (6-9V via HAT VIN)?
2. **ArUco marker size** - Larger = detectable from farther. What is the toy car size?
3. **Camera calibration** - Need checkerboard calibration for accurate ArUco pose depth
4. **Fork material** - 3D printed, bent aluminum, or craft/hobby material?
5. **Home position** - Does the robot return to a fixed start point after picking up?

---

## Software Stack
- Language: **Python 3.11**
- Camera: `picamera2`
- Vision: `opencv-python`, `cv2.aruco` (ArUco detection + pose estimation)
- Motors: `adafruit-circuitpython-motorkit` (Motor HAT via I2C)
- Servo (arm): `RPi.GPIO` PWM or Motor HAT servo channel
- Main loop: threaded (camera thread + control loop thread)

---

## File Structure (planned)
```
senior_design_final/
├── plan.md                  ← this file
├── src/
│   ├── main.py              ← entry point / state machine
│   ├── detection/
│   │   └── aruco_detector.py   ← ArUco detect + pose estimation
│   ├── navigation/
│   │   └── controller.py    ← PID steering + approach logic
│   ├── motors/
│   │   └── driver.py        ← Motor HAT abstraction (drive + arm servo)
│   └── camera/
│       └── capture.py       ← picamera2 stream (threaded)
├── tests/
├── requirements.txt
└── README.md
```

---

## Next Steps (immediate)
1. **Answer arm question** - What kind of lift mechanism? Scoop, claw, or forklift-style?
2. Assemble chassis (motors, wheels, caster, Motor HAT, Pi)
3. Get picamera2 feed working with OpenCV on the Pi
4. Implement ArUco detection proof-of-concept + print test marker
5. Basic motor spin test via Motor HAT
6. Build + tune PID navigation loop
