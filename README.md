# AI Valet Robot

Autonomous robot that retrieves toy cars from parking spots and delivers them to the exit.

---

## Track Layout

```
  (Green Hexagon)                   (Green Square)
      PS2                               PS1
       │                                 │
       │ black tape                      │ blue tape
       │                                 │
       └──────────────┬──────────────────┘
                      │ fork (black + blue junction)
                      │ black tape
                      │
  (Green Circle) ─────┴──────────────── (Green Triangle)
      HOME             black tape            EXIT
```

### Tape Color Key

| Tape Color | Purpose |
|------------|---------|
| Black | Main aisle — robot follows by default |
| Blue | Branch to PS1 — also used by Car 1 after pickup to reach EXIT |

### Stopping Markers (green electrical tape shapes on floor)

All markers are **green tape**. Shape determines the spot.

| Spot | Shape | Notes |
|------|-------|-------|
| Parking Spot 1 (PS1) | Square (4 sides) | Car 1 stops here |
| Parking Spot 2 (PS2) | Hexagon (6 sides) | Car 2 stops here |
| Home | Circle | Mission complete |
| Exit | Triangle (3 sides) | Car is dropped off here |

### Routing After Pickup

| Mission | Line followed to EXIT |
|---------|-----------------------|
| Car 1   | Blue tape |
| Car 2   | Black tape |

Robot lifts when it sees its target shape, stays lifted until EXIT triangle, then lowers.

---

## Missions

### Pick Up Car 1 (from PS1)
```
HOME → (black right) → PS1 branch (blue) → PS1
     → lift car
     → (blue back to black) → (black right) → EXIT
     → lower car
     → (black left) → HOME branch (blue) → HOME
```

### Pick Up Car 2 (from PS2)
```
HOME → (black left) → PS2 branch (blue) → PS2
     → lift car
     → (blue back to black) → (black right) → EXIT
     → lower car
     → (black left) → HOME branch (blue) → HOME
```

---

## Running the Robot

### Option A – Web UI (recommended)

**On the Pi** — start the robot inside a tmux session named `valet`:

```bash
tmux new-session -s valet
cd ~/senior_design_final
source .venv/bin/activate
python -m src.main
# Detach with Ctrl-B D  (session keeps running)
```

**On your laptop** — start the web server:

```bash
cd ui
pip install flask
python server.py --host <pi-ip> --user will
# Open http://localhost:5000
```

The web UI sends commands by SSH-ing into the Pi and using `tmux send-keys` to inject keypresses into the `valet` session. The robot session must be running inside tmux for this to work.

### Option B – SSH keyboard control

```bash
ssh will@<pi-ip>
cd senior_design_final
source .venv/bin/activate
python -m src.main
```

Key controls:
| Key | Action |
|-----|--------|
| `1` | Pick up Car 1 from PS1 → deliver to EXIT |
| `2` | Pick up Car 2 from PS2 → deliver to EXIT |
| `h` | Return robot to HOME |
| `SPACE` | Emergency stop |
| `q` | Quit |

---

## Hardware

| Component | Details |
|-----------|---------|
| Brain | Raspberry Pi 4B |
| Camera | USB webcam (facing DOWN for line following) |
| Chassis | Adafruit Mini Robot Rover 2WD |
| Motor Driver | Adafruit Motor HAT (I2C) |
| Lift Servo | FS90 micro servo — raises platform under car |
| Power | USB-C power bank (Pi) + 6x AA batteries (motors) |

---

## Lift Servo Wiring (2x FS90, in sync)

Two FS90 servos raise/lower the lift platform together.
Connect through the Motor HAT GPIO passthrough header.

```
Motor HAT GPIO passthrough (top of HAT)

Pin  2  ──── 5V  ──────┬──── Servo 1 VCC  (red)
                        └──── Servo 2 VCC  (red)

Pin  6  ──── GND ──────┬──── Servo 1 GND  (brown)
                        └──── Servo 2 GND  (brown)

Pin 12  ──── GPIO 18 ───── Servo 1 Signal (orange)

Pin 13  ──── GPIO 27 ───── Servo 2 Signal (orange)
```

| FS90 | Signal Pin | GPIO |
|------|-----------|------|
| Servo 1 | Pin 12 | GPIO 18 |
| Servo 2 | Pin 13 | GPIO 27 |

Both share 5V (Pin 2) and GND (Pin 6).

**Pulse widths (tune in config.py):**
- `SERVO_PULSE_DOWN_US = 1000` — platform lowered (travel position)
- `SERVO_PULSE_UP_US = 1800` — platform raised (carrying car)

**Test the lift:**
```bash
source .venv/bin/activate
python3 -c "
import sys; sys.path.insert(0, '.')
from src.motors.driver import MotorDriver
import time
m = MotorDriver()
print('Lifting...'); m.lift_up(); time.sleep(2)
print('Lowering...'); m.lift_down(); time.sleep(1)
m.cleanup()
"
```

---

## State Machine

```
         ┌──────────┐
    ──▶  │   IDLE   │  waiting for mission command
         └────┬─────┘
              │ key press (1 or 2)
              ▼
         ┌──────────┐
         │ NAVIGATE │  line following to target spot
         └────┬─────┘
              │ stopping marker detected
              ▼
         ┌──────────┐
         │  LIFTING │  servo raises platform under car
         └────┬─────┘
              │ lift complete
              ▼
         ┌──────────┐
         │ DELIVER  │  line follow to EXIT
         └────┬─────┘
              │ EXIT marker detected
              ▼
         ┌──────────┐
         │  LOWER   │  servo lowers, car released
         └────┬─────┘
              │ lower complete
              ▼
         ┌──────────┐
         │ RETURN   │  line follow back to HOME
         └────┬─────┘
              │ HOME marker detected
              ▼
         ┌──────────┐
         │   IDLE   │  mission complete
         └──────────┘
```

---

## File Structure

```
senior_design_final/
├── README.md
├── COMMANDS.md          ← useful commands for setup/testing
├── WIRING.md            ← hardware wiring guide
├── plan.md              ← project planning notes
├── requirements.txt
└── src/
    ├── main.py          ← entry point + key input handler
    ├── config.py        ← all tunable parameters
    ├── camera/
    │   └── capture.py   ← USB camera stream (threaded)
    ├── detection/
    │   └── aruco_detector.py  ← color/shape + line detection
    ├── navigation/
    │   └── controller.py      ← state machine + line following PID
    └── motors/
        └── driver.py    ← Motor HAT + lift servo abstraction
```
