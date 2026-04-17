"""
Simulation setup utilities.

This module is responsible for:
- Connecting to the CARLA server
- Loading the world
- Spawning the ego vehicle
- Attaching sensors
"""

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

    transform = carla.Transform(carla.Location(x=1.5, z=1.6))
    return world.spawn_actor(cam_bp, transform, attach_to=ego)


def attach_collision_sensor(world, ego):
    """Attach a collision sensor to the ego vehicle."""
    bp_lib = world.get_blueprint_library()
    col_bp = bp_lib.find("sensor.other.collision")
    return world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)
