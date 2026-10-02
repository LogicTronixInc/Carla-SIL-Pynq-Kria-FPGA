#Tesla Model 3 based script

#!/usr/bin/env python3

import carla
import random
import time
import os
import numpy as np

from network_sender_wrapper import NetworkSender, wrap_save_image_network, wrap_save_lidar_network

# ============================================================
# CONFIG
# ============================================================

HOST = "localhost"
PORT = 2000

MAP_NAME = "Town10HD_Opt"

NUM_NPC_VEHICLES = 1

OUTPUT_DIR = "tesla_drive_sim_dataset"

FPS = 20
DELTA_SECONDS = 1.0 / FPS

os.makedirs(OUTPUT_DIR, exist_ok=True)

sender = NetworkSender(dest_ip = "192.168.1.14", dest_port = 9000) #fpga board

CAMERA_SENSOR_IDS = {
    "front": 0, "front_left": 1, "front_right": 2,
    "side_left": 3, "side_right": 4, "rear": 5,
}


# ============================================================
# CONNECT
# ============================================================

client = carla.Client(HOST, PORT)
client.set_timeout(20.0)

available_maps = client.get_available_maps()

print("\nAvailable Maps:")
for m in available_maps:
    print(m)

if any(MAP_NAME in m for m in available_maps):

    print(f"\nLoading map: {MAP_NAME}")

    world = client.load_world(MAP_NAME)

else:

    print(f"\nMap {MAP_NAME} not found.")
    world = client.get_world()

time.sleep(3)

# ============================================================
# WEATHER (DRIVE SIM STYLE DAYLIGHT)
# ============================================================

weather = carla.WeatherParameters(

    cloudiness=10.0,
    precipitation=0.0,
    precipitation_deposits=0.0,
    wind_intensity=5.0,
    fog_density=0.0,
    wetness=0.0,
    sun_altitude_angle=45.0

)

world.set_weather(weather)

# ============================================================
# SYNCHRONOUS MODE
# ============================================================

settings = world.get_settings()

settings.synchronous_mode = True
settings.fixed_delta_seconds = DELTA_SECONDS

world.apply_settings(settings)

traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)

# ============================================================
# BLUEPRINTS
# ============================================================

blueprint_library = world.get_blueprint_library()

premium_vehicle_ids = [

    "vehicle.tesla.model3",
    "vehicle.lincoln.mkz_2020",
    "vehicle.audi.etron",
    "vehicle.mercedes.coupe",
    "vehicle.dodge.charger_2020"

]

vehicle_blueprints = []

for vid in premium_vehicle_ids:

    vehicle_blueprints.extend(
        blueprint_library.filter(vid)
    )

spawn_points = world.get_map().get_spawn_points()

# ============================================================
# TESLA MODEL 3 EGO VEHICLE
# ============================================================

ego_bp = blueprint_library.find(
    "vehicle.tesla.model3"
)

ego_bp.set_attribute("color", "255,255,255")

ego_transform = random.choice(spawn_points)

ego_vehicle = world.try_spawn_actor(
    ego_bp,
    ego_transform
)

if ego_vehicle is None:
    raise RuntimeError("Failed to spawn ego vehicle")

ego_vehicle.set_autopilot(True)

print("\nTesla Model3 spawned")

# ============================================================
# NPC VEHICLES
# ============================================================

npc_vehicles = []

random.shuffle(spawn_points)

for transform in spawn_points[:NUM_NPC_VEHICLES]:

    try:

        bp = random.choice(vehicle_blueprints)

        if bp.has_attribute("color"):

            color = random.choice(
                bp.get_attribute("color").recommended_values
            )

            bp.set_attribute("color", color)

        npc = world.try_spawn_actor(bp, transform)

        if npc is not None:

            npc.set_autopilot(True)

            traffic_manager.distance_to_leading_vehicle(
                npc,
                3.0
            )

            npc_vehicles.append(npc)

    except:
        pass

print(f"Spawned {len(npc_vehicles)} NPC vehicles")

# ============================================================
# SENSOR STORAGE
# ============================================================

sensor_list = []

def save_image(image, folder):

    folder_path = os.path.join(OUTPUT_DIR, folder)

    os.makedirs(folder_path, exist_ok=True)

    image.save_to_disk(
        os.path.join(
            folder_path,
            f"{image.frame:06d}.png"
        )
    )

def save_lidar(point_cloud):

    folder_path = os.path.join(
        OUTPUT_DIR,
        "lidar"
    )

    os.makedirs(folder_path, exist_ok=True)

    points = np.frombuffer(
        point_cloud.raw_data,
        dtype=np.float32
    )

    points = np.reshape(points, (-1, 4))

    np.save(
        os.path.join(
            folder_path,
            f"{point_cloud.frame:06d}.npy"
        ),
        points
    )

# ============================================================
# LIDAR
# ============================================================

lidar_bp = blueprint_library.find(
    "sensor.lidar.ray_cast"
)

lidar_bp.set_attribute("channels", "64")
lidar_bp.set_attribute("range", "120")
lidar_bp.set_attribute("points_per_second", "500000")
lidar_bp.set_attribute("rotation_frequency", "20")
lidar_bp.set_attribute("upper_fov", "2.0")
lidar_bp.set_attribute("lower_fov", "-24.8")
lidar_bp.set_attribute("sensor_tick", str(DELTA_SECONDS))

lidar_transform = carla.Transform(
    carla.Location(x=0.0, y=0.0, z=2.5)
)

lidar_sensor = world.spawn_actor(
    lidar_bp,
    lidar_transform,
    attach_to=ego_vehicle
)

# lidar_sensor.listen(save_lidar)
lidar_sensor.listen(
    # save_lidar
    lambda point_cloud: wrap_save_lidar_network(point_cloud, save_lidar, sender)
)

sensor_list.append(lidar_sensor)

print("64-channel LiDAR attached")

# ============================================================
# CAMERA CONFIGURATION
# ============================================================

def configure_camera(bp, width, height, fov):

    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", str(fov))
    bp.set_attribute("sensor_tick", str(DELTA_SECONDS))

    # BIGGEST VISUAL UPGRADE
    bp.set_attribute("enable_postprocess_effects", "True")

    bp.set_attribute("motion_blur_intensity", "0")
    bp.set_attribute("motion_blur_max_distortion", "0")

    bp.set_attribute("chromatic_aberration_intensity", "0")

    bp.set_attribute("lens_circle_falloff", "0")
    bp.set_attribute("lens_circle_multiplier", "0")

    bp.set_attribute("gamma", "2.2")

    bp.set_attribute("shutter_speed", "200")

    bp.set_attribute("iso", "100")

    bp.set_attribute("fstop", "1.8")

# ============================================================
# FRONT LONG RANGE CAMERA
# ============================================================

front_cam_bp = blueprint_library.find(
    "sensor.camera.rgb"
)

configure_camera(
    front_cam_bp,
    640,
    420,
    90
)

front_camera = world.spawn_actor(

    front_cam_bp,

    carla.Transform(
        carla.Location(
            x=2.5,
            z=2.0
        )
    ),

    attach_to=ego_vehicle
)

front_camera.listen(
    # lambda image: save_image(image, "front_camera")
    # lambda image: wrap_save_image(image, "front_camera", CAMERA_SENSOR_IDS["front"], save_image, pcap_writer)
    lambda image: wrap_save_image_network(image, "front_camera", CAMERA_SENSOR_IDS["front"], save_image, sender)
)

sensor_list.append(front_camera)

print("Front camera attached")

# ============================================================
# SURROUND CAMERA RIG
# ============================================================

camera_setup = {

    "front_left": -60,
    "front_right": 60,
    "side_left": -120,
    "side_right": 120,
    "rear": 180
}

# for name, yaw in camera_setup.items():

#     cam_bp = blueprint_library.find(
#         "sensor.camera.rgb"
#     )

#     configure_camera(
#         cam_bp,
#         720,
#         480,
#         90
#     )

#     cam_transform = carla.Transform(

#         carla.Location(
#             x=0.0,
#             y=0.0,
#             z=2.1
#         ),

#         carla.Rotation(yaw=yaw)
#     )

#     cam = world.spawn_actor(
#         cam_bp,
#         cam_transform,
#         attach_to=ego_vehicle
#     )

#     cam.listen(
#         lambda image, n=name:
#         save_image(image, n)
#     )

#     # # cam.listen(
#     # #     lambda image, n=name: wrap_save_image(image, n, CAMERA_SENSOR_IDS[n], save_image, pcap_writer)
#     # # )
#     # cam.listen(
#     #     lambda image, n=name: wrap_save_image_network(image, n, CAMERA_SENSOR_IDS[n], save_image, sender)
#     # )

#     sensor_list.append(cam)

#     print(f"{name} camera attached")

# ============================================================
# TRAFFIC SETTINGS
# ============================================================

traffic_manager.global_percentage_speed_difference(5)

for npc in npc_vehicles:

    traffic_manager.auto_lane_change(
        npc,
        True
    )

# ============================================================
# SPECTATOR CAMERA
# ============================================================

spectator = world.get_spectator()

# ============================================================
# MAIN LOOP
# ============================================================

try:

    print("\nSimulation running at 5 FPS...\n")

    frame = 0

    #my changes

    last_tick_time = time.time()
    #######

    while True:
        world.tick()

        now = time.time()

        print(f"tick took {now - last_tick_time:.3f}s")
        last_tick_time = now

        transform = ego_vehicle.get_transform()

        location = transform.location
        rotation = transform.rotation

        # BETTER CINEMATIC SPECTATOR CAMERA

        spectator_transform = carla.Transform(

            location + carla.Location(
                x=-7,
                z=3
            ),

            carla.Rotation(
                pitch=-12,
                yaw=rotation.yaw
            )
        )

        spectator.set_transform(
            # spectator_transform
            front_camera.get_transform()
        )

        print(
            f"Frame {frame} | "
            f"Location: "
            f"({location.x:.2f}, "
            f"{location.y:.2f}, "
            f"{location.z:.2f})"
        )

        frame += 1

except KeyboardInterrupt:

    print("\nStopping simulation")

finally:

    print("\nCleaning up actors...")

    for sensor in sensor_list:

        sensor.stop()
        sensor.destroy()

    for npc in npc_vehicles:
        npc.destroy()

    ego_vehicle.destroy()

    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None

    world.apply_settings(settings)

    print("Cleanup complete")

