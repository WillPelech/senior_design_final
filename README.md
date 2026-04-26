# AI Valet Robot

Autonomous robot that retrieves toy cars from parking spots and delivers them to the exit.

---

## Track Layout

```
        PS2            HOME           PS1          EXIT
     (Red Square)  (Green Circle) (Blue Square)  (Yellow Triangle)
         │               │               │               │
         │  (blue tape)  │  (blue tape)  │  (blue tape)  │  (black tape)
         │               │               │               │
─────────┴───────────────┴───────────────┴───────────────┴──────────────
                         BLACK TAPE MAIN AISLE
```

### Tape Color Key

| Tape Color | Purpose |
|------------|---------|
| Black | Main aisle — robot follows this to travel left/right |
| Blue | Branch lines — splits off black to enter/exit each car spot |

### Stopping Markers (colored electrical tape shapes on white floor)

| Spot | Shape | Tape Color |
|------|-------|------------|
| Parking Spot 1 (right) | Square | Blue |
| Parking Spot 2 (left) | Square | Red |
| Robot Home (center) | Circle | Green |
| Exit / Entrance (far right) | Triangle | Yellow |

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

## SSH Control UI

Connect via SSH then run:

```bash
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
| Lift Servo | SG90/MG90S — raises platform under car |
| Power | USB-C power bank (Pi) + 6x AA batteries (motors) |

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
