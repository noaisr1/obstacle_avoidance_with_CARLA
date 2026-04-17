"""
Global configuration file for the obstacle avoidance project.

This version uses CARLA built-in autopilot (no 'agents' dependency).
The Safety Supervisor can override autopilot control with emergency braking.
"""

from pathlib import Path

# Path to CARLA installation (WindowsNoEditor folder)
CARLA_ROOT = Path(r"C:\CARLA\WindowsNoEditor")

# CARLA server connection
HOST = "127.0.0.1"
PORT = 2000

# Map name (set to None to keep current map)
MAP_NAME = None  # Example: "Town10HD_Opt"

# Camera parameters
IMG_W = 800
IMG_H = 450
FOV = 90

# Region Of Interest (ROI) in normalized image coordinates
ROI_X_MIN = 0.40
ROI_X_MAX = 0.60
ROI_Y_MIN = 0.35
ROI_Y_MAX = 0.80

# Obstacle detection threshold:
# percentage of ROI pixels classified as vehicle or pedestrian
OBSTACLE_RATIO_THRESHOLD = 0.06

# Emergency braking strength
EMERGENCY_BRAKE = 1.0

# Experiment switches:
# - BASELINE: ego uses autopilot only (no supervisor override)
# - SUPERVISOR: ego uses autopilot + supervisor override on risk
RUN_MODE = "SUPERVISOR"  # "BASELINE" or "SUPERVISOR"

# Run time limit (seconds). Stop run after this duration even if no destination exists.
RUN_TIME_LIMIT_SEC = 90

# Logging interval (seconds)
LOG_DT_SEC = 0.05
