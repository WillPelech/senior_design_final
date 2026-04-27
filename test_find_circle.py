"""
Test: spin until green circle is found, drive toward it, stop when close.
Run: python test_find_circle.py
Press SPACE to stop, q to quit.
"""

import sys
import time
import cv2
import numpy as np

sys.path.insert(0, '.')
from src.motors.driver import MotorDriver

# ── Tunable ──────────────────────────────────────────────────────────────────
SPIN_SPEED      = 0.15   # motor speed while searching
DRIVE_SPEED     = 0.20   # motor speed while approaching
STEER_KP        = 0.003  # proportional steering gain
CLOSE_AREA      = 40000  # stop when green blob area exceeds this (px²)

GREEN_HSV_LOW   = np.array([35,  30,  80])
GREEN_HSV_HIGH  = np.array([85, 255, 255])
MIN_DETECT_AREA = 3000   # minimum area to consider a detection valid
# ─────────────────────────────────────────────────────────────────────────────

motors = MotorDriver()
cap    = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 0)

print("Starting — robot will spin to find green circle then drive toward it.")
print("SPACE = stop   q = quit")

state = "SEARCH"   # SEARCH | APPROACH | DONE

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        h, w = frame.shape[:2]
        cx_frame = w / 2.0

        # ── Detect green circle ──────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, GREEN_HSV_LOW, GREEN_HSV_HIGH)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_DETECT_AREA:
                continue
            # Check circularity
            perim = cv2.arcLength(cnt, True)
            if perim == 0:
                continue
            circularity = 4 * np.pi * area / (perim ** 2)
            if circularity > 0.55 and area > best_area:
                best = cnt
                best_area = area

        # ── Display ─────────────────────────────────────────────────────────
        display = cv2.resize(frame, (960, 540))
        ds = 960 / w
        status_text = f"State: {state}"

        if best is not None:
            bx, by, bw, bh = cv2.boundingRect(best)
            cx_blob = bx + bw / 2.0
            x_err   = cx_blob - cx_frame

            cv2.rectangle(display,
                          (int(bx*ds), int(by*ds)),
                          (int((bx+bw)*ds), int((by+bh)*ds)),
                          (0, 255, 0), 2)
            cv2.putText(display, f"CIRCLE area={int(best_area)} err={x_err:+.0f}",
                        (int(bx*ds), int(by*ds)-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            # ── State machine ────────────────────────────────────────────────
            if state == "SEARCH":
                print(f"Circle found! area={int(best_area)}  driving toward it...")
                state = "APPROACH"

            if state == "APPROACH":
                if best_area >= CLOSE_AREA:
                    print("Reached circle — stopping.")
                    motors.stop()
                    state = "DONE"
                else:
                    correction = STEER_KP * x_err
                    left  = max(0.15, min(1.0, DRIVE_SPEED + correction))
                    right = max(0.15, min(1.0, DRIVE_SPEED - correction))
                    motors.set_motors(left, right)

        else:
            if state == "APPROACH":
                print("Lost circle — resuming search...")
                state = "SEARCH"
            if state == "SEARCH":
                motors.set_motors(SPIN_SPEED, -SPIN_SPEED)

        cv2.putText(display, status_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv2.putText(display, f"area={int(best_area)}  target={CLOSE_AREA}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        cv2.imshow("Find Circle", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            print("STOP")
            motors.stop()
            state = "DONE"

        if state == "DONE":
            time.sleep(0.05)

finally:
    motors.stop()
    motors.cleanup()
    cap.release()
    cv2.destroyAllWindows()
    print("Done.")
