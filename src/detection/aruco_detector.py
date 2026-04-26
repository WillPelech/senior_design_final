# =============================================================================
# detection/aruco_detector.py  –  Line following + stopping marker detection
# =============================================================================
# Detects:
#   - Black tape line position (for steering PID)
#   - Blue fork tape (signals junction between PS1 and PS2 branches)
#   - Colored stopping markers at each spot:
#       PS1  = blue square
#       PS2  = red square
#       HOME = green circle
#       EXIT = yellow triangle
# =============================================================================

import logging
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from src import config

log = logging.getLogger(__name__)


@dataclass
class DetectionResult:
    """
    Result of one detection pass on a single frame.

    line_found:   True if black line is visible
    line_x_error: pixels from center (positive = line is RIGHT of center)
    fork_detected: True if blue fork tape is visible
    at_ps1:       True if PS1 blue square marker detected
    at_ps2:       True if PS2 red square marker detected
    at_home:      True if HOME green circle marker detected
    at_exit:      True if EXIT yellow triangle marker detected
    annotated:    frame with detection overlays drawn
    """
    line_found:    bool  = False
    line_x_error:  float = 0.0
    fork_detected: bool  = False
    at_ps1:        bool  = False
    at_ps2:        bool  = False
    at_home:       bool  = False
    at_exit:       bool  = False
    annotated:     Optional[np.ndarray] = None

    # kept for interface compatibility with motor driver / telemetry
    found_car:  bool  = False
    found_home: bool  = False
    distance:   float = 0.0
    x_error:    float = 0.0
    all_ids:    list  = field(default_factory=list)


class ArucoDetector:
    """
    Line + marker detector for downward-facing camera.
    Call detect(frame) each loop tick.
    """

    def __init__(self) -> None:
        self._cx = config.CAMERA_WIDTH / 2.0
        self._fork_counter = 0
        log.info("LineDetector ready")

    def detect(self, frame: np.ndarray) -> DetectionResult:
        result = DetectionResult()
        if frame is None:
            return result

        annotated = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        self._detect_line(gray, annotated, result)
        self._detect_fork(hsv, annotated, result)
        self._detect_markers(hsv, annotated, result)

        # keep legacy fields in sync
        result.x_error    = result.line_x_error
        result.found_home = result.at_home

        result.annotated = annotated
        return result

    # ------------------------------------------------------------------
    # Line detection
    # ------------------------------------------------------------------

    def _detect_line(self, gray, annotated, result):
        """Find black tape line center at LINE_SAMPLE_ROW."""
        row = config.LINE_SAMPLE_ROW
        if row >= gray.shape[0]:
            return

        # Threshold: black tape = low value
        _, thresh = cv2.threshold(gray, config.LINE_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        # Sample a horizontal strip around the scan row
        strip = thresh[max(0, row - 10):row + 10, :]
        col_sum = np.sum(strip, axis=0)

        # Find contiguous white (black tape) regions
        line_pixels = np.where(col_sum > 0)[0]
        if len(line_pixels) == 0:
            return

        # Group into contiguous segments
        segments = []
        seg_start = line_pixels[0]
        prev = line_pixels[0]
        for px in line_pixels[1:]:
            if px - prev > 5:
                segments.append((seg_start, prev))
                seg_start = px
            prev = px
        segments.append((seg_start, prev))

        # Pick widest segment within allowed width range
        best = None
        for seg in segments:
            w = seg[1] - seg[0]
            if config.LINE_MIN_WIDTH_PX <= w <= config.LINE_MAX_WIDTH_PX:
                if best is None or w > (best[1] - best[0]):
                    best = seg

        if best is None:
            return

        line_cx = (best[0] + best[1]) / 2.0
        result.line_found   = True
        result.line_x_error = line_cx - self._cx

        # Annotate
        cv2.line(annotated, (0, row), (annotated.shape[1], row), (0, 255, 255), 1)
        cv2.circle(annotated, (int(line_cx), row), 6, (0, 255, 0), -1)
        cv2.line(annotated, (int(self._cx), row - 10), (int(self._cx), row + 10), (255, 255, 255), 1)
        cv2.putText(annotated, f"line x_err={result.line_x_error:+.0f}px",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ------------------------------------------------------------------
    # Fork detection (blue tape junction)
    # ------------------------------------------------------------------

    def _detect_fork(self, hsv, annotated, result):
        """Detect blue tape indicating the PS1/PS2 fork."""
        mask = cv2.inRange(hsv, np.array(config.BLUE_HSV_LOW), np.array(config.BLUE_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        area = cv2.countNonZero(mask)
        if area >= config.FORK_MIN_AREA_PX:
            self._fork_counter += 1
        else:
            self._fork_counter = max(0, self._fork_counter - 1)

        if self._fork_counter >= config.FORK_CONFIRM_FRAMES:
            result.fork_detected = True
            cv2.putText(annotated, "FORK DETECTED", (10, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)

    # ------------------------------------------------------------------
    # Stopping marker detection
    # ------------------------------------------------------------------

    def _detect_markers(self, hsv, annotated, result):
        result.at_ps1  = self._detect_blue_square(hsv, annotated)
        result.at_ps2  = self._detect_red_square(hsv, annotated)
        result.at_home = self._detect_green_circle(hsv, annotated)
        result.at_exit = self._detect_yellow_triangle(hsv, annotated)

    def _detect_blue_square(self, hsv, annotated) -> bool:
        mask = cv2.inRange(hsv, np.array(config.PS1_HSV_LOW), np.array(config.PS1_HSV_HIGH))
        return self._check_square(mask, annotated, "PS1", (255, 100, 0))

    def _detect_red_square(self, hsv, annotated) -> bool:
        m1 = cv2.inRange(hsv, np.array(config.PS2_HSV_LOW1), np.array(config.PS2_HSV_HIGH1))
        m2 = cv2.inRange(hsv, np.array(config.PS2_HSV_LOW2), np.array(config.PS2_HSV_HIGH2))
        mask = cv2.bitwise_or(m1, m2)
        return self._check_square(mask, annotated, "PS2", (0, 0, 255))

    def _detect_green_circle(self, hsv, annotated) -> bool:
        mask = cv2.inRange(hsv, np.array(config.HOME_HSV_LOW), np.array(config.HOME_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < config.MIN_MARKER_AREA:
            return False
        perimeter = cv2.arcLength(largest, True)
        if perimeter == 0:
            return False
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < config.CIRCLE_CIRCULARITY_MIN:
            return False
        (cx, cy), r = cv2.minEnclosingCircle(largest)
        cv2.circle(annotated, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
        cv2.putText(annotated, "HOME", (int(cx) - 20, int(cy) - int(r) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return True

    def _detect_yellow_triangle(self, hsv, annotated) -> bool:
        mask = cv2.inRange(hsv, np.array(config.EXIT_HSV_LOW), np.array(config.EXIT_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < config.MIN_MARKER_AREA:
            return False
        approx = cv2.approxPolyDP(largest, 0.04 * cv2.arcLength(largest, True), True)
        if len(approx) != 3:
            return False
        x, y, w, h = cv2.boundingRect(largest)
        cv2.drawContours(annotated, [approx], -1, (0, 255, 255), 2)
        cv2.putText(annotated, "EXIT", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        return True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _check_square(self, mask, annotated, label, color) -> bool:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < config.MIN_MARKER_AREA:
            return False
        approx = cv2.approxPolyDP(largest, 0.04 * cv2.arcLength(largest, True), True)
        if len(approx) != 4:
            return False
        x, y, w, h = cv2.boundingRect(largest)
        aspect = w / h if h > 0 else 0
        if not (config.SQUARE_ASPECT_MIN < aspect < config.SQUARE_ASPECT_MAX):
            return False
        cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
        cv2.putText(annotated, label, (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return True
