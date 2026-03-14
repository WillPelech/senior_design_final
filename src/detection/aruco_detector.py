# =============================================================================
# detection/aruco_detector.py  –  ArUco marker detection + pose estimation
# =============================================================================
# Detects ArUco markers in a BGR frame and returns their 3D pose relative to
# the camera. The pose translation vector (tvec) is the key output:
#
#   tvec[0]  = lateral offset in meters  (+right, -left from camera center)
#   tvec[1]  = vertical offset in meters (+down, -up)
#   tvec[2]  = depth / distance in meters (how far the marker is)
#
# All tunable parameters (dict type, marker ID, physical size, calibration)
# come from config.py.
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

# ---------------------------------------------------------------------------
# ArUco dictionary name → cv2 constant mapping
# ---------------------------------------------------------------------------
_ARUCO_DICT_MAP = {
    "DICT_4X4_50":   cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":  cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50":   cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100":  cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250":  cv2.aruco.DICT_6X6_250,
}


@dataclass
class DetectionResult:
    """
    Result of one detection attempt on a single frame.

    Attributes:
        found_car:   True if the car marker (ARUCO_CAR_MARKER_ID) was detected
        found_home:  True if the home marker (ARUCO_HOME_MARKER_ID) was detected
        car_tvec:    3-element array [x, y, z] in meters (car position)
        car_rvec:    3-element rotation vector (Rodrigues) for the car marker
        home_tvec:   3-element array [x, y, z] in meters (home position)
        home_rvec:   rotation vector for the home marker
        x_error:     lateral offset of car from camera center-line (meters)
        distance:    depth to car marker (tvec[2]) in meters
        all_ids:     list of all detected marker IDs in this frame
        annotated:   copy of the frame with ArUco overlays drawn (or None)
    """
    found_car:   bool = False
    found_home:  bool = False
    car_tvec:    Optional[np.ndarray] = None
    car_rvec:    Optional[np.ndarray] = None
    home_tvec:   Optional[np.ndarray] = None
    home_rvec:   Optional[np.ndarray] = None
    x_error:     float = 0.0   # meters; positive = car is to the RIGHT
    distance:    float = 0.0   # meters to car marker
    all_ids:     list = field(default_factory=list)
    annotated:   Optional[np.ndarray] = None


class ArucoDetector:
    """
    Wraps OpenCV's ArUco detection + pose estimation pipeline.

    Call detect(frame) each loop tick to get a DetectionResult.
    The detector is stateless – it does not track history across frames.
    History / lost-frame counting is handled by the navigation controller.
    """

    def __init__(self) -> None:
        # Build camera matrix and distortion array from config lists
        self._camera_matrix = np.array(config.CAMERA_MATRIX, dtype=np.float64)
        self._dist_coeffs   = np.array(config.CAMERA_DIST_COEFFS, dtype=np.float64)

        # Resolve dictionary constant from name string in config
        dict_id = _ARUCO_DICT_MAP.get(config.ARUCO_DICT_NAME)
        if dict_id is None:
            raise ValueError(
                f"Unknown ARUCO_DICT_NAME '{config.ARUCO_DICT_NAME}'. "
                f"Valid options: {list(_ARUCO_DICT_MAP.keys())}"
            )
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        # Detection parameters – expose key ones through config
        params = cv2.aruco.DetectorParameters()
        params.minMarkerPerimeterRate = config.ARUCO_MIN_PERIMETER_RATE
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, params)

        # Half-width of the frame (pixels) used for x_error normalisation
        self._cx = config.CAMERA_WIDTH / 2.0   # principal point x (approx)

        log.info(
            "ArucoDetector ready | dict=%s | car_id=%d | home_id=%s | marker=%.3fm",
            config.ARUCO_DICT_NAME,
            config.ARUCO_CAR_MARKER_ID,
            str(config.ARUCO_HOME_MARKER_ID),
            config.ARUCO_MARKER_SIZE_M,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, frame: np.ndarray) -> DetectionResult:
        """
        Run ArUco detection on a single BGR frame.

        Args:
            frame: H×W×3 BGR numpy array from the camera.

        Returns:
            DetectionResult with pose information for car and/or home markers.
        """
        result = DetectionResult()

        if frame is None:
            return result

        # Convert to greyscale for detection (faster, same accuracy)
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = self._detector.detectMarkers(grey)

        if ids is None or len(ids) == 0:
            # No markers found – return empty result
            if config.DEBUG_SHOW_PREVIEW:
                result.annotated = frame.copy()
            return result

        result.all_ids = ids.flatten().tolist()

        # Draw all detected markers on a copy of the frame
        annotated = frame.copy()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        # Estimate pose for every detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            config.ARUCO_MARKER_SIZE_M,
            self._camera_matrix,
            self._dist_coeffs,
        )

        for i, marker_id in enumerate(result.all_ids):
            tvec = tvecs[i][0]  # shape (3,)  [x, y, z]
            rvec = rvecs[i][0]  # shape (3,)  Rodrigues rotation

            # Draw pose axes on the annotated frame
            if config.DEBUG_DRAW_AXES:
                cv2.drawFrameAxes(
                    annotated,
                    self._camera_matrix,
                    self._dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    config.ARUCO_MARKER_SIZE_M * 0.5,  # axis length = half marker
                )

            if marker_id == config.ARUCO_CAR_MARKER_ID:
                result.found_car = True
                result.car_tvec  = tvec
                result.car_rvec  = rvec
                result.distance  = float(tvec[2])
                # x_error: positive = car is to the RIGHT of center
                # tvec[0] from solvePnP is already in camera frame meters
                result.x_error = float(tvec[0])

                # Overlay text: distance and x-offset
                center = self._marker_center(corners[i])
                self._draw_label(
                    annotated, center,
                    f"CAR  d={tvec[2]:.2f}m  x={tvec[0]:+.3f}m",
                    color=(0, 255, 0),
                )

            elif marker_id == config.ARUCO_HOME_MARKER_ID:
                result.found_home = True
                result.home_tvec  = tvec
                result.home_rvec  = rvec

                center = self._marker_center(corners[i])
                self._draw_label(
                    annotated, center,
                    f"HOME d={tvec[2]:.2f}m",
                    color=(255, 128, 0),
                )

        result.annotated = annotated
        return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _marker_center(corner: np.ndarray) -> tuple:
        """Return the pixel (cx, cy) of a detected marker's bounding box center."""
        pts = corner[0]  # shape (4, 2)
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        return (cx, cy)

    @staticmethod
    def _draw_label(
        img: np.ndarray,
        center: tuple,
        text: str,
        color: tuple = (0, 255, 0),
    ) -> None:
        """Draw a small text label near a marker center."""
        x, y = center
        cv2.putText(
            img, text,
            (x - 60, y - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )
