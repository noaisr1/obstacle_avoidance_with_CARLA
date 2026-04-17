"""
Safety supervisor module.

Decides whether to override vehicle control
based on the perceived obstacle ratio.
"""

import carla
from config import OBSTACLE_RATIO_THRESHOLD, EMERGENCY_BRAKE


class SafetySupervisor:
    def __init__(self):
        self.threshold = OBSTACLE_RATIO_THRESHOLD
        self.brake_value = EMERGENCY_BRAKE

    def decide(self, obstacle_ratio):
        """
        Decide whether to trigger emergency braking.

        Returns:
            (override: bool, control: carla.VehicleControl or None)
        """
        if obstacle_ratio >= self.threshold:
            control = carla.VehicleControl(
                throttle=0.0,
                brake=self.brake_value,
                steer=0.0
            )
            return True, control

        return False, None
