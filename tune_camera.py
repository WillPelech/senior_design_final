"""
Camera tuning script — streams live feed at http://<pi-ip>:8081

Open in browser while SSH'd in to see the annotated feed.
Type keys in the SSH terminal to adjust values (no Enter needed).

Controls:
  z/x   zoom out / zoom in
  +/-   LINE_THRESHOLD
  a/d   green H min/max
  r/f   green S min
  q     quit and print final settings
"""

import cv2
import numpy as np
import sys
import select
import termios
import tty
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

# --- Tunable values ---
line_threshold = 160
g_h_min, g_h_max = 35, 85
g_s_min = 30
g_v_min = 80
zoom = 1.0

STREAM_PORT    = 8081
DISPLAY_WIDTH  = 640
DISPLAY_HEIGHT = 360

# --- MJPEG stream ---
_frame = [None]
_lock  = threading.Lock()


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            self.connection.settimeout(15)
            while True:
                with _lock:
                    f = _frame[0]
                if f is not None:
                    _, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    data = jpg.tobytes()
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(data)}\r\n\r\n".encode()
                        + data + b"\r\n"
                    )
                    self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass


def _start_stream():
    server = HTTPServer(("0.0.0.0", STREAM_PORT), _Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Stream live at http://<pi-ip>:{STREAM_PORT}")


# --- Non-blocking key reader ---
def _setup_tty():
    fd = sys.stdin.fileno()
    try:
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        return fd, old
    except Exception:
        return fd, None


def _read_key(fd):
    try:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            return sys.stdin.read(1)
    except Exception:
        pass
    return ""


def _restore_tty(fd, old):
    if old:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            pass


# --- Main ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 0)

_start_stream()

fd, old_tty = _setup_tty()

print("Controls: z/x=zoom  +/-=threshold  a/d=H  r/f=Smin  q=quit")
print("Center HSV values will print every second.\n")

last_print = 0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        h_orig, w_orig = frame.shape[:2]

        # Zoom crop
        crop_h = int(h_orig * zoom)
        crop_w = int(w_orig * zoom)
        y0 = (h_orig - crop_h) // 2
        x0 = (w_orig - crop_w) // 2
        frame = frame[y0:y0+crop_h, x0:x0+crop_w]
        h_orig, w_orig = frame.shape[:2]

        display = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
        scale_x = DISPLAY_WIDTH  / w_orig
        scale_y = DISPLAY_HEIGHT / h_orig

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Black line scan
        scan_row = int(h_orig * 0.75)
        _, thresh = cv2.threshold(gray, line_threshold, 255, cv2.THRESH_BINARY_INV)
        strip = thresh[scan_row-10:scan_row+10, :]
        col_sum = np.sum(strip, axis=0)
        line_pixels = np.where(col_sum > 0)[0]
        scan_row_d = int(scan_row * scale_y)
        cv2.line(display, (0, scan_row_d), (DISPLAY_WIDTH, scan_row_d), (0, 255, 255), 1)
        if len(line_pixels) > 0:
            cx = int(line_pixels.mean() * scale_x)
            cv2.circle(display, (cx, scan_row_d), 8, (0, 255, 0), -1)

        # Green shape detection
        g_mask = cv2.inRange(hsv,
                             np.array([g_h_min, g_s_min, g_v_min]),
                             np.array([g_h_max, 255, 255]))
        g_mask = cv2.morphologyEx(g_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(g_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 3000:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            x2, y2 = int(x*scale_x), int(y*scale_y)
            w2, h2 = int(w*scale_x), int(h*scale_y)
            cv2.rectangle(display, (x2, y2), (x2+w2, y2+h2), (0, 200, 0), 2)
            cv2.putText(display, f"area={int(area)}", (x2, y2-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)

        # Center crosshair + HSV
        cy_orig, cx_orig = h_orig // 2, w_orig // 2
        hsv_center = hsv[cy_orig, cx_orig]
        cv2.circle(display, (DISPLAY_WIDTH//2, DISPLAY_HEIGHT//2), 6, (255, 255, 255), -1)

        # HUD overlay
        lines = [
            f"ZOOM={zoom:.2f}  Frame:{w_orig}x{h_orig}",
            f"LINE threshold={line_threshold}  found={'YES' if len(line_pixels)>0 else 'NO'}",
            f"GREEN H=[{g_h_min},{g_h_max}] S_min={g_s_min}",
            f"Center HSV: H={hsv_center[0]} S={hsv_center[1]} V={hsv_center[2]}",
        ]
        for i, txt in enumerate(lines):
            cv2.putText(display, txt, (10, 18 + i*20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        with _lock:
            _frame[0] = display

        # Print HSV to terminal every second
        now = time.monotonic()
        if now - last_print >= 1.0:
            print(f"  Center HSV: H={hsv_center[0]:3d} S={hsv_center[1]:3d} V={hsv_center[2]:3d}  "
                  f"GREEN H=[{g_h_min},{g_h_max}] S_min={g_s_min}  LINE={line_threshold}")
            last_print = now

        # Key input
        key = _read_key(fd)
        if key == 'q':
            break
        elif key == 'z':
            zoom = max(0.2, zoom - 0.05)
        elif key == 'x':
            zoom = min(1.0, zoom + 0.05)
        elif key in ('+', '='):
            line_threshold = min(254, line_threshold + 5)
        elif key == '-':
            line_threshold = max(1, line_threshold - 5)
        elif key == 'd':
            g_h_max = min(179, g_h_max + 2)
        elif key == 'a':
            g_h_min = max(0, g_h_min - 2)
        elif key == 'r':
            g_s_min = min(254, g_s_min + 5)
        elif key == 'f':
            g_s_min = max(0, g_s_min - 5)

finally:
    _restore_tty(fd, old_tty)
    cap.release()

print(f"\nFinal settings:")
print(f"  LINE_THRESHOLD  = {line_threshold}")
print(f"  MARKER_HSV_LOW  = [{g_h_min}, {g_s_min}, {g_v_min}]")
print(f"  MARKER_HSV_HIGH = [{g_h_max}, 255, 255]")
