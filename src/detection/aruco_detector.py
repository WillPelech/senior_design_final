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
    # Colour-based detection — each spot is a uniquely coloured square
    #   HOME  = green   PS1 = grey   PS2 = red   EXIT = purple
    # ------------------------------------------------------------------

    def _detect_all_shapes(self, hsv, annotated, result):
        # Green → HOME
        self._detect_color(hsv, annotated, result, 'home',
                           config.HOME_HSV_LOW, config.HOME_HSV_HIGH,
                           (0, 220, 0))

        # Purple → EXIT
        self._detect_color(hsv, annotated, result, 'exit',
                           config.EXIT_HSV_LOW, config.EXIT_HSV_HIGH,
                           (200, 0, 220))

        # Blue → PS1 (Car 1)
        self._detect_color(hsv, annotated, result, 'ps1',
                           config.PS1_HSV_LOW, config.PS1_HSV_HIGH,
                           (255, 140, 0))

        # Red → PS2 (Car 2) — red wraps around HSV, combine two ranges
        mask_r1 = cv2.inRange(hsv, np.array(config.PS2_HSV_LOW_1), np.array(config.PS2_HSV_HIGH_1))
        mask_r2 = cv2.inRange(hsv, np.array(config.PS2_HSV_LOW_2), np.array(config.PS2_HSV_HIGH_2))
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        self._detect_mask(red_mask, annotated, result, 'ps2', (0, 0, 220))

    def _detect_color(self, hsv, annotated, result, name, hsv_low, hsv_high, color):
        mask = cv2.inRange(hsv, np.array(hsv_low), np.array(hsv_high))
        self._detect_mask(mask, annotated, result, name, color)

    def _detect_mask(self, mask, annotated, result, name, color):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_cnt  = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < config.SHAPE_MIN_AREA or area <= best_area:
                continue
            # Confirm roughly rectangular: filled area vs bounding box area
            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = w * h
            if bbox_area == 0:
                continue
            fill_ratio = area / bbox_area
            if fill_ratio < 0.45:  # reject blobs that aren't roughly rectangular
                continue
            best_area = area
            best_cnt  = cnt

        if best_cnt is None:
            return

        x, y, w, h = cv2.boundingRect(best_cnt)
        x_err = (x + w / 2.0) - self._cx

        setattr(result, f'{name}_found',   True)
        setattr(result, f'{name}_x_error', x_err)
        setattr(result, f'{name}_area',    best_area)

        cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
        cv2.putText(annotated, f"{name.upper()} err={x_err:+.0f} a={best_area:.0f}",
                    (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
