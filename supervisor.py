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
    EMERGENCY_BRAKE,
)

class SafetySupervisor:
    def __init__(self):
        self.brake_value = EMERGENCY_BRAKE

    def decide(self, bev_map, speed_mps):
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
        
        # Define the lane ROI at the bottom of the BEV map (ego position).
        # The road area in the BEV is mapped to x in [0.20*w, 0.80*w] by IPM_DST_POINTS_NORM.
        # The ROI is centered inside that mapped road band rather than blindly at the image center,
        # so it always covers the full drivable area even when the ego is not dead-center.
        road_x_min = int(0.20 * w)
        road_x_max = int(0.80 * w)
        road_center = (road_x_min + road_x_max) // 2
        lane_half = min(ROI_LANE_WIDTH_PX // 2, (road_x_max - road_x_min) // 2)
        x_start = max(road_x_min, road_center - lane_half)
        x_end = min(road_x_max, road_center + lane_half)
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