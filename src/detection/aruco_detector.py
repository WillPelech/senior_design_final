# =============================================================================
# detection/aruco_detector.py  –  Color + shape detection
# =============================================================================
# Detects colored shapes in a BGR frame:
#   Blue circle   → car
#   Red square    → home / parking spot
#   Green triangle → start position (informational)
#
# Distance is estimated from the apparent size of the detected shape using
# the pinhole camera model:  distance = (focal_length * real_size) / pixel_size
#
# All tunable parameters (HSV ranges, real sizes) come from config.py.
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
    Result of one detection attempt on a single frame.

    Attributes:
        found_car:   True if the blue circle (car) was detected
        found_home:  True if the red square (parking spot) was detected
        found_start: True if the green triangle (start) was detected
        x_error:     lateral offset of car from camera center-line (meters)
        distance:    estimated distance to car (meters)
        all_ids:     unused, kept for interface compatibility
        annotated:   copy of the frame with detection overlays drawn
    """
    found_car:   bool = False
    found_home:  bool = False
    found_start: bool = False
    car_tvec:    Optional[np.ndarray] = None
    car_rvec:    Optional[np.ndarray] = None
    home_tvec:   Optional[np.ndarray] = None
    home_rvec:   Optional[np.ndarray] = None
    x_error:     float = 0.0
    distance:    float = 0.0
    all_ids:     list = field(default_factory=list)
    annotated:   Optional[np.ndarray] = None


class ArucoDetector:
    """
    Color + shape detector. Replaces ArUco detection.

    Detects:
      - Blue circle  → car
      - Red square   → home/parking spot
      - Green triangle → start

    Call detect(frame) each loop tick to get a DetectionResult.
    """

    def __init__(self) -> None:
        self._fx = config.CAMERA_MATRIX[0][0]   # focal length x (pixels)
        self._cx = config.CAMERA_WIDTH / 2.0

        log.info(
            "ColorShapeDetector ready | car=blue circle | home=red square | start=green triangle"
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, frame: np.ndarray) -> DetectionResult:
        """
        Run color+shape detection on a single BGR frame.

        Args:
            frame: H×W×3 BGR numpy array from the camera.

        Returns:
            DetectionResult with position info for detected shapes.
        """
        result = DetectionResult()

        if frame is None:
            return result

        annotated = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        self._detect_car(hsv, annotated, result)
        self._detect_home(hsv, annotated, result)
        self._detect_start(hsv, annotated, result)

        result.annotated = annotated
        return result

    # ------------------------------------------------------------------
    # Shape detectors
    # ------------------------------------------------------------------

    def _detect_car(self, hsv: np.ndarray, annotated: np.ndarray, result: DetectionResult) -> None:
        """Detect blue circle → car."""
        mask = cv2.inRange(hsv, np.array(config.BLUE_HSV_LOW), np.array(config.BLUE_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < config.MIN_SHAPE_AREA:
            return

        # Check circularity
        perimeter = cv2.arcLength(largest, True)
        if perimeter == 0:
            return
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < config.CIRCLE_CIRCULARITY_MIN:
            return

        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        cx, cy, radius = int(cx), int(cy), int(radius)

        # Estimate distance using apparent radius
        if radius > 0:
            result.distance = (self._fx * config.CAR_REAL_RADIUS_M) / radius
        result.x_error = (cx - self._cx) / self._fx * result.distance
        result.found_car = True

        # Annotate
        cv2.circle(annotated, (cx, cy), radius, (255, 0, 0), 2)
        cv2.circle(annotated, (cx, cy), 4, (255, 0, 0), -1)
        cv2.putText(
            annotated,
            f"CAR  d={result.distance:.2f}m  x={result.x_error:+.3f}m",
            (cx - 60, cy - radius - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv2.LINE_AA,
        )

    def _detect_home(self, hsv: np.ndarray, annotated: np.ndarray, result: DetectionResult) -> None:
        """Detect red square → home/parking spot."""
        # Red wraps around 0/180 in HSV so combine two ranges
        mask1 = cv2.inRange(hsv, np.array(config.RED_HSV_LOW1), np.array(config.RED_HSV_HIGH1))
        mask2 = cv2.inRange(hsv, np.array(config.RED_HSV_LOW2), np.array(config.RED_HSV_HIGH2))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < config.MIN_SHAPE_AREA:
            return

        # Check squareness via approx polygon
        approx = cv2.approxPolyDP(largest, 0.04 * cv2.arcLength(largest, True), True)
        if len(approx) != 4:
            return

        x, y, w, h = cv2.boundingRect(largest)
        aspect = w / h if h > 0 else 0
        if not (config.SQUARE_ASPECT_MIN < aspect < config.SQUARE_ASPECT_MAX):
            return

        cx = x + w // 2
        cy = y + h // 2
        result.found_home = True
        # Store home position as tvec-like for controller compatibility
        dist = (self._fx * config.HOME_REAL_SIZE_M) / max(w, h) if max(w, h) > 0 else 0
        x_err = (cx - self._cx) / self._fx * dist
        result.home_tvec = np.array([x_err, 0.0, dist])

        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(
            annotated,
            f"HOME d={dist:.2f}m",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA,
        )

    def _detect_start(self, hsv: np.ndarray, annotated: np.ndarray, result: DetectionResult) -> None:
        """Detect green triangle → start position."""
        mask = cv2.inRange(hsv, np.array(config.GREEN_HSV_LOW), np.array(config.GREEN_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < config.MIN_SHAPE_AREA:
            return

        approx = cv2.approxPolyDP(largest, 0.04 * cv2.arcLength(largest, True), True)
        if len(approx) != 3:
            return

        x, y, w, h = cv2.boundingRect(largest)
        result.found_start = True

        cv2.drawContours(annotated, [approx], -1, (0, 255, 0), 2)
        cv2.putText(
            annotated,
            "START",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA,
        )
