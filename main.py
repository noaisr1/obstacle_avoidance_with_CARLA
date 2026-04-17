"""
Main simulation loop for the CARLA Obstacle Avoidance project.
Coordinates sensors, perception, and safety supervision.
Updated with:
- Synchronous Mode (for smooth camera and deterministic results)
- Automatic Chase Cam (Spectator tracking)
- Dynamic ROI Safety Logic
"""

import carla
import cv2
import os
import numpy as np
from config import (
    IMG_W, IMG_H, FOV, RUN_MODE, RUN_TIME_LIMIT_SEC, LOG_DT_SEC,
    NPC_VEHICLE_COUNT, TM_PORT, DEBUG_OUTPUT_DIR
)
from sim_setup import (
    connect, get_world, spawn_ego,
    attach_segmentation_camera, attach_collision_sensor,
    spawn_npc_vehicles,
)
from perception import SegmentationPerception
from supervisor import SafetySupervisor
from metrics import MetricsLogger


def get_speed(vehicle):
    """Calculate vehicle speed (m/s)."""
    v = vehicle.get_velocity()
    return (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5


def main():
    # 1. Connection and World Setup
    client = connect()
    world = get_world(client)
    
    # Enable Synchronous Mode for smooth physics and camera
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = LOG_DT_SEC # Matches our target Hz
    world.apply_settings(settings)

    # Setup Traffic Manager in Sync Mode
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)

    spectator = world.get_spectator()

    # 2. Spawn Ego and NPCs
    ego, _ = spawn_ego(world)
    npc_list = spawn_npc_vehicles(world, n=NPC_VEHICLE_COUNT, tm_port=TM_PORT)
    
    # Start driving
    ego.set_autopilot(True, TM_PORT)
    autopilot_enabled = True

    # 3. Sensor Setup
    perception = SegmentationPerception(IMG_W, IMG_H)
    camera = attach_segmentation_camera(world, ego, IMG_W, IMG_H, FOV)
    camera.listen(lambda image: perception.on_image(image))

    supervisor = SafetySupervisor()
    metrics = MetricsLogger("run_metrics.csv")

    collision_sensor = attach_collision_sensor(world, ego)
    collision_sensor.listen(lambda event: metrics.on_collision(event))

    # DEBUG_OUTPUT_DIR is a pathlib.Path in config.py
    try:
        DEBUG_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    except AttributeError:
        # Fallback if DEBUG_OUTPUT_DIR is ever changed to a string
        os.makedirs(DEBUG_OUTPUT_DIR, exist_ok=True)

    print(f"[main] Simulation starting in SYNC mode ({RUN_MODE})...")
    
    # Hysteresis for stability
    clear_streak = 0
    clear_streak_required = 10 
    frames_elapsed = 0
    max_frames = int(RUN_TIME_LIMIT_SEC / LOG_DT_SEC)

    try:
        while frames_elapsed < max_frames:
            # Advance simulation
            world.tick()
            frames_elapsed += 1
            
            # --- Spectator Follow Logic (Chase Cam) ---
            ego_tf = ego.get_transform()
            v_forward = ego_tf.get_forward_vector()
            # Position: 10m back, 5m up
            spec_loc = ego_tf.location - v_forward * 10 + carla.Location(z=5)
            spec_rot = carla.Rotation(pitch=-20, yaw=ego_tf.rotation.yaw, roll=0)
            spectator.set_transform(carla.Transform(spec_loc, spec_rot))

            # --- Perception & Control ---
            bev_frame = perception.get_bev_map()
            bev_visual = perception.last_bev_visual
            speed = get_speed(ego)

            # Debug Visualization
            if bev_visual is not None:
                cv2.imshow("Safety Layer - Bird's Eye View", bev_visual)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            override = False
            roi_lookahead_px = 0
            min_obstacle_dist_px = None

            if RUN_MODE.upper() == "SUPERVISOR":
                override, control, roi_lookahead_px, min_obstacle_dist_px = supervisor.decide(bev_frame, speed)
                
                if override:
                    clear_streak = 0
                    if autopilot_enabled:
                        ego.set_autopilot(False)
                        autopilot_enabled = False
                    ego.apply_control(carla.VehicleControl(brake=1.0, throttle=0.0))
                else:
                    clear_streak += 1
                    if not autopilot_enabled and clear_streak >= clear_streak_required:
                        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
                        ego.set_autopilot(True, TM_PORT)
                        autopilot_enabled = True

            # --- Metrics Logging ---
            metrics.log_step(
                time_sec=frames_elapsed * LOG_DT_SEC,
                frame=world.get_snapshot().frame,
                speed_mps=speed,
                obstacle_ratio=0.0, # Using IPM binary check now
                override=int(override),
                brake=1.0 if override else 0.0,
                collision=metrics.consume_collision_flag(),
                roi_lookahead_px=roi_lookahead_px,
                min_obstacle_dist_px=min_obstacle_dist_px,
            )

    except KeyboardInterrupt:
        print("[main] Stopped by user.")
    finally:
        # Cleanup
        print("[main] Cleaning up simulation...")
        # Disable sync mode to avoid freezing the editor
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
        camera.stop()
        collision_sensor.stop()
        ego.destroy()
        for npc in npc_list:
            try:
                npc.destroy()
            except:
                pass
        cv2.destroyAllWindows()
        metrics.close()
        print(f"[main] Done. Metrics saved to run_metrics.csv")

if __name__ == "__main__":
    main()