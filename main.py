"""
Main entry point for the obstacle avoidance project (no CARLA agents).

This script:
- Connects to CARLA
- Spawns an ego vehicle
- Enables CARLA built-in autopilot for baseline driving
- Attaches a semantic segmentation camera and collision sensor
- Computes an obstacle ratio in a forward ROI
- Uses a Safety Supervisor to override autopilot with emergency braking
- Logs metrics to CSV for evaluation

RUN_MODE controls whether the supervisor is active:
- BASELINE: autopilot only
- SUPERVISOR: autopilot + supervisor override
"""

import math
import time

from config import (
    IMG_W, IMG_H, FOV,
    ROI_X_MIN, ROI_X_MAX, ROI_Y_MIN, ROI_Y_MAX,
    RUN_MODE, RUN_TIME_LIMIT_SEC, LOG_DT_SEC
)
from sim_setup import (
    connect, get_world, spawn_ego,
    attach_segmentation_camera, attach_collision_sensor
)
from perception import SegmentationPerception
from supervisor import SafetySupervisor
from metrics import MetricsLogger


def vector_length(v):
    """Compute Euclidean length of a CARLA vector."""
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def main():
    # Connect and load world
    client = connect()
    world = get_world(client)

    # Spawn ego vehicle
    ego, _spawn_points = spawn_ego(world)

    # Enable built-in autopilot (Traffic Manager handles navigation)
    ego.set_autopilot(True)

    # Attach sensors
    camera = attach_segmentation_camera(world, ego, IMG_W, IMG_H, FOV)
    collision_sensor = attach_collision_sensor(world, ego)

    # Perception + decision modules
    perception = SegmentationPerception(
        IMG_W, IMG_H,
        (ROI_X_MIN, ROI_X_MAX, ROI_Y_MIN, ROI_Y_MAX)
    )
    supervisor = SafetySupervisor()

    # Metrics
    metrics = MetricsLogger("run_metrics.csv")

    # Start sensors
    camera.listen(perception.on_image)
    collision_sensor.listen(metrics.on_collision)

    print(f"RUN_MODE = {RUN_MODE}")
    print("Autopilot enabled. Stop with Ctrl+C. Output: run_metrics.csv")

    start = time.time()

    try:
        while True:
            now = time.time()
            elapsed = now - start

            # Stop after fixed time (simple, stable experiment design)
            if elapsed >= RUN_TIME_LIMIT_SEC:
                print("Time limit reached.")
                break

            # Latest perception results
            frame, ratio = perception.get_obstacle_ratio()

            # Ego speed
            speed = vector_length(ego.get_velocity())

            # Decide whether to override autopilot
            override = False
            brake_val = 0.0

            if RUN_MODE.upper() == "SUPERVISOR":
                override, override_control = supervisor.decide(ratio)
                if override:
                    ego.apply_control(override_control)
                    brake_val = override_control.brake

            # Collision marker (best-effort based on frame id)
            collision = int(metrics.last_collision_frame == frame and frame != -1)

            metrics.log_step(
                time_sec=elapsed,
                frame=frame,
                speed_mps=speed,
                obstacle_ratio=ratio,
                override=override,
                brake=brake_val,
                collision=collision
            )

            time.sleep(LOG_DT_SEC)

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        # Stop sensors first
        try:
            camera.stop()
        except Exception:
            pass

        metrics.close()

        # Destroy actors
        for actor in [collision_sensor, camera, ego]:
            try:
                actor.destroy()
            except Exception:
                pass

        print(f"Finished. Collisions: {metrics.collision_count}, "
              f"Emergency overrides: {metrics.emergency_override_count}. "
              f"CSV saved: run_metrics.csv")


if __name__ == "__main__":
    main()
