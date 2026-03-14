# AI Valet Robot – Wiring & Hardware Setup

Everything you need to physically build and connect the robot before running any code.

---

## Parts List

| # | Part | Notes |
|---|------|-------|
| 1 | Raspberry Pi 4B | Brain |
| 2 | Adafruit Mini Robot Rover Chassis Kit #2939 | Frame, 2x DC motors, caster, wheels |
| 3 | Adafruit DC & Stepper Motor HAT for Raspberry Pi #2348 | Motor controller, stacks on Pi GPIO header |
| 4 | Raspberry Pi Camera Module (v2 or v3) | CSI ribbon cable included |
| 5 | MG90S or SG90 micro servo | Forklift lift servo |
| 6 | 6x AA battery holder with switch | Powers motors via HAT (9V nominal) |
| 7 | 6x AA batteries (alkaline) | Motor power source |
| 8 | USB-C power bank (5V, ≥ 2A) | Powers the Pi separately |
| 9 | USB-C cable | Pi power |
| 10 | Jumper wires (female-to-female and female-to-male) | Signal wiring |
| 11 | Small zip ties or double-sided tape | Cable management |

> **Why two power sources?**
> The motors draw spikes of current that cause voltage dips on the 5V rail,
> which can crash the Pi. Keeping them on separate power sources prevents this.

---

## Pi 4B GPIO Pinout Reference

```
                    +-------+
               3V3 |  1   2| 5V        ← Pi power input (USB-C, NOT this pin)
         SDA1 (I2C)|  3   4| 5V
         SCL1 (I2C)|  5   6| GND
                   |  7   8|
               GND |  9  10|
                   | 11  12| GPIO18    ← SERVO SIGNAL (config: SERVO_GPIO_PIN = 18)
                   | 13  14| GND       ← SERVO GND
              3V3  | 17  18|
                   | 19  20| GND
                   | 21  22|
                   | 23  24|
               GND | 25  26|
                   | 27  28|
                   | 29  30| GND
                   | 31  32|
                   | 33  34| GND
                   | 35  36|
                   | 37  38|
               GND | 39  40|
                    +-------+
```

The Motor HAT **stacks directly on top of all 40 pins** – you do not need to
plug individual wires into the Pi GPIO header for the motors. I2C (pins 3 & 5)
is used automatically through the HAT stack.

---

## 1. Motor HAT → Pi

**Just stack it.** The Motor HAT plugs directly onto the Pi's 40-pin GPIO header.
No individual wires needed between the HAT and Pi for motor control.

```
Pi GPIO header (all 40 pins)
        ↕  (stack)
Motor HAT connector (all 40 pins)
```

Make sure the HAT is seated firmly and all pins are aligned before powering on.

---

## 2. DC Motors → Motor HAT

The chassis has two DC motors (left and right drive wheels). Each motor has
two bare wire leads. Connect them to the Motor HAT terminal blocks on the top
of the board.

```
Motor HAT terminal blocks (top of board)
┌──────────────────────────────────┐
│  M1+ │ M1- │ M2+ │ M2- │ M3 │ M4│
└──────────────────────────────────┘
```

| Motor | HAT Terminal | Wire Color (typical) |
|-------|-------------|----------------------|
| LEFT motor  – wire A | M1+ | Red |
| LEFT motor  – wire B | M1- | Black |
| RIGHT motor – wire A | M2+ | Red |
| RIGHT motor – wire B | M2- | Black |

> **If a wheel spins backwards:** swap that motor's two wires on the terminal
> block (swap M1+ and M1-). Do NOT change the code – fix it in hardware.
>
> **Motor number in code:** `motor1` = left, `motor2` = right. This matches
> `src/motors/driver.py`. If you wire them the other way, swap the assignments
> in that file.

---

## 3. Battery Pack → Motor HAT (Motor Power)

The Motor HAT has a dedicated 2-pin power terminal block labeled **PWR / VIN**
(or **+** and **–**) separate from the motor terminals. This is the motor
power input.

```
Motor HAT power terminal (usually top-right or bottom-right corner)
┌──────────┐
│  + │  –  │
└──────────┘
    ↑    ↑
  RED  BLACK  from battery holder
```

| Battery holder wire | Motor HAT terminal |
|--------------------|--------------------|
| Red   (+) | VIN / + |
| Black (–) | GND / – |

**Battery voltage:** 6x AA alkaline = ~9V fully charged, ~7.2V when depleted.
The Motor HAT accepts 5–12V on the VIN terminal. Do not exceed 12V.

**Use the battery holder's built-in switch** to cut motor power without
unplugging anything.

> The Motor HAT does NOT power the Pi from VIN – they are isolated. The Pi
> must still be powered separately via USB-C.

---

## 4. Pi Power

Power the Pi 4B with a **USB-C power bank** rated at **5V / 2A or higher**.
Plug it into the USB-C port on the Pi (the leftmost USB-C port, not the
USB-A ports).

Do not try to power the Pi from the Motor HAT or from the AA batteries.

---

## 5. Pi Camera → Pi

Connect the Pi Camera Module to the **CSI camera port** on the Pi using the
included flat ribbon cable.

```
Pi 4B board
┌──────────────────────┐
│  [CSI Camera Port]   │  ← ribbon cable goes here
│  (between HDMI ports │
│   and USB ports)     │
└──────────────────────┘
```

**Ribbon cable orientation:**
- The blue plastic tab faces the USB ports (away from the HDMI ports)
- Gently lift the black plastic latch, insert the ribbon, press the latch down

Mount the camera at the **front of the robot** facing forward, at the same
height as the ArUco marker on the toy car (or slightly above). Angling it
slightly downward helps when the robot is very close to the target.

---

## 6. Gripper Servo → Pi GPIO

The gripper is the **Thingiverse thing:2415 Mini Servo Gripper by jjshortcut**,
driven by a single SG90 or MG90S micro servo. The servo has three wires.
Connect them to the Pi GPIO header **on top of the Motor HAT** (the HAT passes
all 40 pins through).

| Servo wire | Color (typical) | Pi Pin | Label |
|-----------|----------------|--------|-------|
| Signal    | Orange / Yellow | Pin 12 | GPIO 18 |
| Power (+) | Red            | Pin 2  | 5V |
| Ground    | Brown / Black  | Pin 6  | GND |

> **Use 5V (Pin 2) for the gripper servo**, not 3.3V. The gripper mechanism
> has more mechanical load than a bare servo horn - 3.3V may cause it to
> twitch or stall under grip force. The SG90/MG90S is rated 4.8-6V.

**GPIO pin in config:** `SERVO_GPIO_PIN = 18` in `src/config.py`. Change this
number if you wire the servo signal to a different GPIO pin.

```
Pi GPIO header (looking down at the top of the Motor HAT passthrough)

Pin  2  ──── 5V      ──── Servo VCC    (red)
Pin  6  ──── GND     ──── Servo GND    (brown/black)
Pin 12  ──── GPIO 18 ──── Servo SIGNAL (orange/yellow)
```

### Gripper mounting

- Print thing:2415 from Thingiverse in PLA (0.2mm layer height, 20% infill)
- Mount the servo body into the gripper frame as the design specifies
- Mount the whole gripper assembly to the **front of the chassis top plate**
  using M3 bolts through the chassis holes or with double-sided foam tape
- The gripper jaw center-line should sit at **~18-22mm off the ground** –
  roughly the height of a Hot Wheels car body. Use a small riser block
  (3D printed or a stack of washers) under the gripper base if needed
- The ArUco marker on the toy car should face the camera above the gripper

---

## 7. Full Wiring Diagram (text)

```
┌─────────────────────────────────────────────────────┐
│                  Raspberry Pi 4B                    │
│                                                     │
│  [USB-C] ←── Power Bank (5V/2A+)                   │
│  [CSI]   ←── Pi Camera Module (ribbon)              │
│  [GPIO]  ──── (all 40 pins) ───┐                    │
└──────────────────────────────┬─┘                    │
                               │ (stacked)            │
┌──────────────────────────────▼──────────────────────┤
│              Adafruit Motor HAT #2348                │
│                                                     │
│  M1+ ←── LEFT  motor wire A (red)                  │
│  M1- ←── LEFT  motor wire B (black)                │
│  M2+ ←── RIGHT motor wire A (red)                  │
│  M2- ←── RIGHT motor wire B (black)                │
│                                                     │
│  VIN ←── Battery holder RED  (+)                   │
│  GND ←── Battery holder BLACK (-)                  │
│                                                     │
│  GPIO passthrough pins:                             │
│    Pin  2 (5V)     ──── Gripper Servo VCC  (red)   │
│    Pin  6 (GND)    ──── Gripper Servo GND  (brown) │
│    Pin 12 (GPIO18) ──── Gripper Servo SIG  (orange)│
└─────────────────────────────────────────────────────┘

Battery holder ──── 6x AA alkaline (9V)
  Red   → Motor HAT VIN
  Black → Motor HAT GND
  Switch → inline (use to cut motor power)

Power bank ──── USB-C → Pi (5V/2A+)
```

---

## 8. Mechanical Assembly Order

Follow this order to avoid having to re-do steps:

1. **Assemble the chassis base** – attach motors and caster per Adafruit instructions
2. **Mount wheels** on motor shafts
3. **Attach top plate** with standoffs
4. **Mount Pi** on top plate with M2.5 standoffs (keeps it off the metal)
5. **Stack Motor HAT** onto Pi GPIO header
6. **Wire motors** to Motor HAT terminal blocks
7. **Wire battery holder** to Motor HAT VIN/GND
8. **Mount camera** at front of chassis – use a small bracket or zip tie
9. **Connect camera ribbon** to Pi CSI port
10. **Mount gripper** (thing:2415) on front of chassis at car-body height
11. **Wire servo** to GPIO pins 2, 6, 12 (passthrough on Motor HAT)
12. **Route and secure all cables** with zip ties

---

## 9. Pre-Power Checklist

Go through this before turning anything on:

- [ ] Motor HAT fully seated on all 40 Pi GPIO pins (no bent pins)
- [ ] Left motor wired to M1, right motor to M2
- [ ] Battery holder wired to Motor HAT VIN/GND (NOT to Pi)
- [ ] Battery holder switch is **OFF**
- [ ] Pi Camera ribbon seated and latch closed
- [ ] Gripper servo signal wire on GPIO 18 (Pin 12)
- [ ] Gripper servo GND connected to Pi GND (Pin 6)
- [ ] Gripper servo VCC on 5V (Pin 2)
- [ ] No bare wires touching each other or the metal chassis
- [ ] Power bank plugged into Pi USB-C

**Power on order:**
1. Plug in power bank to Pi (Pi boots)
2. SSH in or connect keyboard/monitor
3. Flip battery holder switch ON (motors now powered)
4. Run `python3 src/main.py`

**Power off order:**
1. Stop the script (q key or Ctrl+C)
2. Flip battery holder switch OFF first
3. Unplug power bank

---

## 10. Common Wiring Mistakes

| Symptom | Likely cause |
|---------|-------------|
| One wheel spins wrong direction | Swap that motor's two wires on the HAT terminal |
| Both wheels spin wrong direction | Swap M1+/M1- AND M2+/M2- or flip `MOTOR_BASE_SPEED` sign in config |
| Motors don't spin at all | Check battery switch is ON; check VIN wired to HAT not Pi |
| Pi crashes when motors run | Motors and Pi sharing power – use separate power bank for Pi |
| Gripper twitches / weak | Check VCC is on 5V (Pin 2) not 3.3V; check battery is charged |
| Gripper doesn't move | Check signal wire is on GPIO 18 / Pin 12; check `SERVO_GPIO_PIN = 18` in config |
| Gripper grinds / binds | `SERVO_PULSE_GRIP_US` is too high – reduce it in config.py |
| Camera not detected | Ribbon not fully seated; latch not closed; run `vcgencmd get_camera` to verify |
| Robot drifts left/right | Motors not matched; tune `STEER_KP` in config or physically swap a motor wire |
