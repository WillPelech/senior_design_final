# =============================================================================
# detection/aruco_detector.py  –  Line following + stopping marker detection
# =============================================================================
# Detects:
#   - Black tape line position (for steering PID)
#   - Blue fork tape (signals junction between PS1 and PS2 branches)
#   - Colored stopping markers at each spot:
#       PS1  = blue square
#       PS2  = red hexagon
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
    Camera faces forward. Shapes are on walls/targets.

    Each shape has: _found (bool), _x_error (px from center), _area (px²)
    Shapes:
      home  = green circle  (HOME)
      ps1   = green square  (Parking Spot 1)
      ps2   = green hexagon (Parking Spot 2)
      exit  = green triangle (EXIT)
    """
    home_found:   bool  = False
    home_x_error: float = 0.0
    home_area:    float = 0.0

    ps1_found:    bool  = False
    ps1_x_error:  float = 0.0
    ps1_area:     float = 0.0

    ps2_found:    bool  = False
    ps2_x_error:  float = 0.0
    ps2_area:     float = 0.0

    exit_found:   bool  = False
    exit_x_error: float = 0.0
    exit_area:    float = 0.0

    annotated:    Optional[np.ndarray] = None

    # legacy compat
    at_ps1:    bool  = False
    at_ps2:    bool  = False
    at_home:   bool  = False
    at_exit:   bool  = False
    line_found: bool = False
    line_x_error: float = 0.0
    fork_detected: bool = False
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
        self._cx = config.CAMERA_WIDTH / 2.0   # updated on first frame
        self._fork_counter = 0
        self._frame_h = config.CAMERA_HEIGHT
        self._frame_w = config.CAMERA_WIDTH
        log.info("LineDetector ready")

    def detect(self, frame: np.ndarray) -> DetectionResult:
        result = DetectionResult()
        if frame is None:
            return result

        # Update center/dimensions from actual frame size
        self._frame_h, self._frame_w = frame.shape[:2]
        self._cx = self._frame_w / 2.0

        annotated = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        self._detect_all_shapes(hsv, annotated, result)

        # keep legacy fields in sync
        result.at_home   = result.home_found
        result.at_ps1    = result.ps1_found
        result.at_ps2    = result.ps2_found
        result.at_exit   = result.exit_found
        result.found_home = result.home_found

        result.annotated = annotated
        return result

    # ------------------------------------------------------------------
    # Shape detection (forward-facing camera, shapes on walls)
    # ------------------------------------------------------------------

    def _detect_all_shapes(self, hsv, annotated, result):
        mask = cv2.inRange(hsv,
                           np.array(config.MARKER_HSV_LOW),
                           np.array(config.MARKER_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < config.SHAPE_MIN_AREA:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            approx   = cv2.approxPolyDP(cnt, 0.03 * perimeter, True)
            sides    = len(approx)
            circularity = 4 * np.pi * area / (perimeter ** 2)

            x, y, w, h = cv2.boundingRect(cnt)
            cx_shape  = x + w / 2.0
            x_err     = cx_shape - self._cx

            # Circle  — high circularity
            if circularity >= config.CIRCLE_CIRCULARITY_MIN:
                if area > result.home_area:
                    result.home_found   = True
                    result.home_x_error = x_err
                    result.home_area    = area
                (ecx, ecy), r = cv2.minEnclosingCircle(cnt)
                cv2.circle(annotated, (int(ecx), int(ecy)), int(r), (0, 255, 0), 2)
                cv2.putText(annotated, f"HOME err={x_err:+.0f}",
                            (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Triangle — 3 sides
            elif sides == 3:
                if area > result.exit_area:
                    result.exit_found   = True
                    result.exit_x_error = x_err
                    result.exit_area    = area
                cv2.drawContours(annotated, [approx], -1, (0, 255, 255), 2)
                cv2.putText(annotated, f"EXIT err={x_err:+.0f}",
                            (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Square — 4 sides, roughly equal aspect
            elif sides == 4:
                aspect = w / h if h > 0 else 0
                if config.SQUARE_ASPECT_MIN < aspect < config.SQUARE_ASPECT_MAX:
                    if area > result.ps1_area:
                        result.ps1_found   = True
                        result.ps1_x_error = x_err
                        result.ps1_area    = area
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 200, 255), 2)
                    cv2.putText(annotated, f"PS1 err={x_err:+.0f}",
                                (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

            # Hexagon — 5-7 sides
            elif 5 <= sides <= 7:
                if area > result.ps2_area:
                    result.ps2_found   = True
                    result.ps2_x_error = x_err
                    result.ps2_area    = area
                cv2.drawContours(annotated, [approx], -1, (255, 100, 0), 2)
                cv2.putText(annotated, f"PS2 err={x_err:+.0f}",
                            (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 1)
