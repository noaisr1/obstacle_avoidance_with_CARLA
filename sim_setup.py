"""
Simulation setup utilities.

This module is responsible for:
- Connecting to the CARLA server
- Loading the world
- Spawning the ego vehicle
- Attaching sensors
"""

import random
import carla
from config import HOST, PORT, MAP_NAME


def connect():
    """Create and return a CARLA client."""
    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)
    return client


def get_world(client):
    """Load a specific map or return the current world."""
    if MAP_NAME:
        return client.load_world(MAP_NAME)
    return client.get_world()


def spawn_ego(world):
    """
    Spawn the ego vehicle at the first available spawn point.
    Returns the vehicle actor and the list of spawn points.
    """
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    if not spawn_points:
        raise RuntimeError("No spawn points found in map")

    vehicle_bps = bp_lib.filter("vehicle.*model3*")
    ego_bp = vehicle_bps[0] if vehicle_bps else bp_lib.filter("vehicle.*")[0]

    for sp in spawn_points:
        ego = world.try_spawn_actor(ego_bp, sp)
        if ego is not None:
            return ego, spawn_points

    raise RuntimeError("Failed to spawn ego vehicle")


def attach_segmentation_camera(world, ego, w, h, fov):
    """Attach a semantic segmentation camera to the ego vehicle."""
    bp_lib = world.get_blueprint_library()
    cam_bp = bp_lib.find("sensor.camera.semantic_segmentation")

    cam_bp.set_attribute("image_size_x", str(w))
    cam_bp.set_attribute("image_size_y", str(h))
    cam_bp.set_attribute("fov", str(fov))

    # Mount the camera at the front of the vehicle with a slight downward pitch.
    # These extrinsics match the thesis parameters (x=1.6, z=1.7, pitch=-15 deg).
    transform = carla.Transform(
        carla.Location(x=1.6, y=0.0, z=1.7),
        carla.Rotation(pitch=-15.0, yaw=0.0, roll=0.0),
    )
    return world.spawn_actor(cam_bp, transform, attach_to=ego)


def spawn_npc_vehicles(world, n=20, tm_port=8000):
    """
    Spawn N NPC vehicles distributed across available spawn points.
    Each NPC is handed to the Traffic Manager so it drives autonomously.
    Returns the list of spawned NPC actors so they can be destroyed at cleanup.
    """
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    vehicle_bps = [
        bp for bp in bp_lib.filter("vehicle.*")
        if int(bp.get_attribute("number_of_wheels")) == 4
    ]

    if not vehicle_bps or not spawn_points:
        print("[sim_setup] Warning: no vehicle blueprints or spawn points found for NPCs.")
        return []

    random.shuffle(spawn_points)
    npc_list = []
    for sp in spawn_points[:n]:
        bp = random.choice(vehicle_bps)
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            actor.set_autopilot(True, tm_port)
            npc_list.append(actor)

    print(f"[sim_setup] Spawned {len(npc_list)} NPC vehicles.")
    return npc_list


def spawn_stopped_vehicle_ahead(world, ego, distance_m=22.0):
    """
    Spawn a stationary obstacle vehicle in the ego's current lane, distance_m ahead.
    Returns the spawned actor or None if spawning fails.
    """
    m = world.get_map()
    ego_tf = ego.get_transform()
    ego_wp = m.get_waypoint(
        ego_tf.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )

    # Walk forward along the lane centerline.
    wp = ego_wp
    remaining = float(distance_m)
    step = 2.0
    while remaining > 0.0:
        nxt = wp.next(min(step, remaining))
        if not nxt:
            break
        wp = nxt[0]
        remaining -= step

    spawn_tf = wp.transform
    spawn_tf.location.z += 0.2  # reduce ground intersection failures

    bp_lib = world.get_blueprint_library()
    # Prefer a noticeable vehicle, but fall back safely.
    candidates = (
        bp_lib.filter("vehicle.*carlacola*")
        or bp_lib.filter("vehicle.*model3*")
        or bp_lib.filter("vehicle.*")
    )
    if not candidates:
        print("[sim_setup] Warning: no vehicle blueprints found for stopped vehicle.")
        return None

    bp = random.choice(candidates)
    stopped = world.try_spawn_actor(bp, spawn_tf)
    if stopped is None:
        print("[sim_setup] Warning: failed to spawn stopped vehicle ahead.")
        return None

    stopped.set_autopilot(False)
    stopped.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
    print(f"[sim_setup] Spawned stopped vehicle {distance_m:.1f} m ahead.")
    return stopped


def attach_collision_sensor(world, ego):
    """Attach a collision sensor to the ego vehicle."""
    bp_lib = world.get_blueprint_library()
    col_bp = bp_lib.find("sensor.other.collision")
    return world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)
