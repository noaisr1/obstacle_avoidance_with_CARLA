"""
Safety Supervisor module.
Implements Dynamic ROI logic on top of the BEV map.
"""

import numpy as np
import carla
from config import (
    ROI_BASE_BUFFER_PX,
    ROI_LOOKAHEAD_MAX_PX,
    ROI_LOOKAHEAD_MIN_PX,
    ROI_LANE_WIDTH_PX,
    ROI_SPEED_FACTOR_PX_PER_MPS,
    ROI_STEER_CENTER_GAIN_PX,
    ROI_STEER_LPF_ALPHA,
    ROI_STEER_WIDTH_GAIN_PX,
    ROI_WIDTH_MAX_PX,
    ROI_WIDTH_MIN_PX,
    EMERGENCY_BRAKE,
)

class SafetySupervisor:
    def __init__(self):
        self.brake_value = EMERGENCY_BRAKE
        self._steer_lpf = 0.0

    def decide(self, bev_map, speed_mps, steer=0.0):
        """
        Analyze the BEV obstacle map with a speed-dependent ROI.

        Academic note: This ROI look-ahead approximates total stopping distance:
            d ≈ v*t + v^2/(2a)
        We model it as a real-time linear pixel look-ahead for robustness and simplicity.

        Returns:
            (override_active, control, roi_lookahead_px, min_obstacle_dist_px)
        """
        if bev_map is None:
            return False, None, 0, None

        h, w = bev_map.shape
        
        # Look-ahead distance (in pixels) = Base_Buffer + (speed_mps * Speed_Factor)
        look_ahead_px = int(ROI_BASE_BUFFER_PX + (speed_mps * ROI_SPEED_FACTOR_PX_PER_MPS))
        look_ahead_px = max(ROI_LOOKAHEAD_MIN_PX, min(look_ahead_px, ROI_LOOKAHEAD_MAX_PX, h))
        
        # Define a steering-aware ROI.
        # - When driving straight, keep ROI narrow to ignore adjacent-lane vehicles.
        # - When changing lanes (|steer| grows), shift ROI center toward the turn direction and widen it.
        road_x_min = int(0.20 * w)
        road_x_max = int(0.80 * w)
        road_center = (road_x_min + road_x_max) // 2

        steer = float(np.clip(steer, -1.0, 1.0))
        self._steer_lpf = (1.0 - ROI_STEER_LPF_ALPHA) * self._steer_lpf + ROI_STEER_LPF_ALPHA * steer

        center_shift = int(self._steer_lpf * ROI_STEER_CENTER_GAIN_PX)
        roi_center = int(np.clip(road_center + center_shift, road_x_min, road_x_max))

        width = int(ROI_WIDTH_MIN_PX + abs(self._steer_lpf) * ROI_STEER_WIDTH_GAIN_PX)
        width = int(np.clip(width, ROI_WIDTH_MIN_PX, ROI_WIDTH_MAX_PX))
        width = min(width, road_x_max - road_x_min)
        half = width // 2

        x_start = max(road_x_min, roi_center - half)
        x_end = min(road_x_max, roi_center + half)
        y_start = max(0, h - look_ahead_px)
        
        roi = bev_map[y_start : h, x_start : x_end]

        min_obstacle_dist_px = None
        if roi.size > 0:
            ys, _xs = np.where(roi == 255)
            if ys.size > 0:
                # ys is relative to roi (0=top of roi). Convert to distance from ego row (bottom).
                # Ego is at row (h-1), ROI bottom aligns with ego row.
                min_obstacle_dist_px = int((roi.shape[0] - 1) - ys.max())

        # If ANY obstacle pixel (value 255) is detected within the dynamic safety zone.
        if min_obstacle_dist_px is not None:
            control = carla.VehicleControl(throttle=0.0, brake=self.brake_value, hand_brake=False)
            return True, control, look_ahead_px, min_obstacle_dist_px

        return False, None, look_ahead_px, None