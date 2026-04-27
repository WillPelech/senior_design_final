"""
Camera tuning script — run on Pi with monitor connected.
Shows live feed at a viewable size with HSV info overlay.

Controls:
  +/-   increase/decrease LINE_THRESHOLD (black line detection)
  w/s   increase/decrease green H max
  a/d   increase/decrease green H min
  r/f   increase/decrease green S min
  q     quit
"""

import cv2
import numpy as np

# --- Tunable values (start from config defaults) ---
line_threshold = 160
g_h_min, g_h_max = 35, 85
g_s_min = 30
g_v_min = 80

DISPLAY_WIDTH  = 960
DISPLAY_HEIGHT = 540

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 0)

print("Camera tuning tool — press q to quit")
print("  +/-  LINE_THRESHOLD")
print("  a/d  green H min/max")
print("  r/f  green S min")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    h_orig, w_orig = frame.shape[:2]

    # Resize for display
    display = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
    scale_x = DISPLAY_WIDTH  / w_orig
    scale_y = DISPLAY_HEIGHT / h_orig

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # --- Black line detection ---
    scan_row_orig = int(h_orig * 0.75)
    _, thresh = cv2.threshold(gray, line_threshold, 255, cv2.THRESH_BINARY_INV)
    strip = thresh[scan_row_orig - 10:scan_row_orig + 10, :]
    col_sum = np.sum(strip, axis=0)
    line_pixels = np.where(col_sum > 0)[0]

    scan_row_disp = int(scan_row_orig * scale_y)
    cv2.line(display, (0, scan_row_disp), (DISPLAY_WIDTH, scan_row_disp), (0, 255, 255), 1)

    line_found = False
    if len(line_pixels) > 0:
        cx = int(line_pixels.mean() * scale_x)
        cv2.circle(display, (cx, scan_row_disp), 8, (0, 255, 0), -1)
        line_found = True

    # --- Green shape detection ---
    g_mask = cv2.inRange(hsv,
                         np.array([g_h_min, g_s_min, g_v_min]),
                         np.array([g_h_max, 255,     255]))
    g_mask = cv2.morphologyEx(g_mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    contours, _ = cv2.findContours(g_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:
            continue
        approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
        sides = len(approx)
        x, y, w, h = cv2.boundingRect(cnt)
        # Scale to display
        x2 = int(x * scale_x); y2 = int(y * scale_y)
        w2 = int(w * scale_x); h2 = int(h * scale_y)
        label = {3: "TRIANGLE", 4: "SQUARE", 5: "PENT", 6: "HEXAGON"}.get(sides, f"{sides}-sides")
        cv2.rectangle(display, (x2, y2), (x2+w2, y2+h2), (0, 200, 0), 2)
        cv2.putText(display, f"{label} area={int(area)}", (x2, y2-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)

    # --- HSV at center ---
    cy_orig, cx_orig = h_orig // 2, w_orig // 2
    hsv_center = hsv[cy_orig, cx_orig]
    cv2.circle(display, (DISPLAY_WIDTH//2, DISPLAY_HEIGHT//2), 5, (255, 255, 255), -1)

    # --- HUD ---
    info = [
        f"LINE threshold={line_threshold}  found={'YES' if line_found else 'NO'}  (+/- to adjust)",
        f"GREEN H=[{g_h_min},{g_h_max}] S_min={g_s_min}  (a/d=H  r/f=Smin)",
        f"Center HSV: H={hsv_center[0]} S={hsv_center[1]} V={hsv_center[2]}",
        f"Frame: {w_orig}x{h_orig}",
    ]
    for i, line in enumerate(info):
        cv2.putText(display, line, (10, 20 + i*20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("Camera Tuner", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif key == ord('+') or key == ord('='):
        line_threshold = min(254, line_threshold + 5)
    elif key == ord('-'):
        line_threshold = max(1, line_threshold - 5)
    elif key == ord('d'):
        g_h_max = min(179, g_h_max + 2)
    elif key == ord('a'):
        g_h_min = max(0, g_h_min - 2)
    elif key == ord('r'):
        g_s_min = min(254, g_s_min + 5)
    elif key == ord('f'):
        g_s_min = max(0, g_s_min - 5)

print(f"\nFinal settings — copy to src/config.py:")
print(f"  LINE_THRESHOLD = {line_threshold}")
print(f"  MARKER_HSV_LOW  = [{g_h_min}, {g_s_min}, {g_v_min}]")
print(f"  MARKER_HSV_HIGH = [{g_h_max}, 255, 255]")

cap.release()
cv2.destroyAllWindows()
