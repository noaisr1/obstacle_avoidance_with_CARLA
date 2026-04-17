"""
Global configuration file for the obstacle avoidance project.
Updated to include Inverse Perspective Mapping (IPM) and Dynamic ROI parameters.
"""

from pathlib import Path

# Output / debug
DEBUG_OUTPUT_DIR = Path("debug_output")
DEBUG_SAVE_BEV_FIRST_N = 5     # save first N frames for quick sanity-check
DEBUG_SAVE_BEV_EVERY_N = 100   # then save one frame every N frames (~5 s at 20 Hz)

# CARLA server connection
HOST = "127.0.0.1"
PORT = 2000
MAP_NAME = None

# Camera parameters
IMG_W = 800
IMG_H = 450
FOV = 90

# Semantic segmentation obstacle label IDs for CARLA 0.9.14+.
# NOTE: CARLA 0.9.14 renumbered all semantic labels.
# Old (<=0.9.13): Pedestrians=4, Vehicles=10  <-- frequently cited but WRONG for 0.9.14+
# Correct (0.9.14+):
#   12 = Pedestrians, 13 = Rider, 14 = Car, 15 = Truck,
#   16 = Bus, 18 = Motorcycle, 19 = Bicycle
OBSTACLE_LABEL_IDS = [12, 13, 14, 15, 16, 18, 19]

# IPM (Inverse Perspective Mapping) points.
# Points are normalized (0.0..1.0) in image coordinates.
#
# Point ordering convention (MUST match for src and dst):
#   [bottom_left, bottom_right, top_right, top_left]
#
# Calibrated via ground-plane projection for:
#   Camera pose: x=1.6 m, z=1.7 m, pitch=-15 deg, FOV=90, W=800, H=450
#   Intrinsics:  f=400, cx=400, cy=225
#
# Ground distances mapped by these points:
#   Near  (y_src=0.65, ~5.5 m)  ->  BEV bottom  (y_dst=0.98)
#   Far   (y_src=0.33, ~25 m)   ->  BEV top     (y_dst=0.02)
# Lane width at near: ±2.5 m (x=0.20..0.80)
# Lane width at far:  ±3.5 m narrows to (x=0.42..0.58)
IPM_SRC_POINTS_NORM = [
    [0.26, 0.65],  # bottom_left  (~5.5 m ahead, ±1.75 m -- ~1 lane width at near field)
    [0.74, 0.65],  # bottom_right (~5.5 m ahead, ±1.75 m)
    [0.57, 0.33],  # top_right    (~25 m  ahead, ±3.5 m)
    [0.43, 0.33],  # top_left     (~25 m  ahead, ±3.5 m)
]

# Destination rectangle (normalized) defining where the road lands in the BEV image.
# Ego vehicle is at the bottom-center; forward direction is toward the top.
IPM_DST_POINTS_NORM = [
    [0.20, 0.98],  # bottom_left
    [0.80, 0.98],  # bottom_right
    [0.80, 0.02],  # top_right
    [0.20, 0.02],  # top_left
]

# Dynamic ROI parameters (distances are in pixels on the BEV map).
# The BEV covers ~5.5 m (bottom) to ~25 m (top) = ~19.5 m over IMG_H pixels.
# Scale: ~0.043 m/px. Stopping distance at 10 m/s ≈ 17 m ≈ 395 px.
# Formula: look_ahead_px = ROI_BASE_BUFFER_PX + speed_mps * ROI_SPEED_FACTOR_PX_PER_MPS
ROI_BASE_BUFFER_PX = 100
ROI_SPEED_FACTOR_PX_PER_MPS = 20       # ≈ 0.86 m per m/s -- tuned for urban speeds
ROI_LANE_WIDTH_PX = 300                 # full road width in BEV (dst spans 0.20-0.80 * 800 = 480 px)
ROI_LOOKAHEAD_MIN_PX = 60
ROI_LOOKAHEAD_MAX_PX = IMG_H - 1       # cap at image height

# ROI lane-awareness (steering-based approximation).
# When driving straight, use a narrower ROI to ignore adjacent-lane traffic.
# When steering strongly (lane-change), shift the ROI center and widen it.
ROI_WIDTH_MIN_PX = 140
ROI_WIDTH_MAX_PX = 360
ROI_STEER_CENTER_GAIN_PX = 220          # pixels of ROI center shift at steer=1.0
ROI_STEER_WIDTH_GAIN_PX = 260           # additional width at steer=1.0
ROI_STEER_LPF_ALPHA = 0.25              # low-pass filter for steer (0..1)

# NPC traffic settings
NPC_VEHICLE_COUNT = 20   # number of NPC vehicles to spawn at startup
TM_PORT = 8000           # Traffic Manager port (must match CARLA server)

# Scenario settings (Option 1: thesis-safe, no overtaking required)
# Spawn a stopped vehicle ahead in the ego lane to trigger the safety layer.
SCENARIO_STOPPED_VEHICLE = True
STOPPED_VEHICLE_DISTANCE_M = 22.0
DISABLE_EXTRA_NPC_TRAFFIC = True

# Controller/behavior toggles
# - For a clean thesis safety demonstration, keep lane changes disabled and latch autopilot off
#   after the first safety trigger (vehicle will stop and stay stopped).
# - For normal driving behavior (bypass/continue), enable lane changes and do not latch.
EGO_ALLOW_LANE_CHANGE = True
LATCH_AUTOPILOT_AFTER_OVERRIDE = False

# End the run automatically once the safety layer achieves a full stop.
STOP_SIM_ON_SUCCESS = False
SUCCESS_STOP_SPEED_MPS = 0.20
SUCCESS_HOLD_TIME_SEC = 1.5

# Simulation settings
EMERGENCY_BRAKE = 1.0
RUN_MODE = "SUPERVISOR" # "BASELINE" or "SUPERVISOR"
RUN_TIME_LIMIT_SEC = 120
LOG_DT_SEC = 0.05