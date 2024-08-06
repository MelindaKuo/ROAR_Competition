from typing import List, Tuple, Dict, Optional
import roar_py_interface
import numpy as np

def normalize_rad(rad: float) -> float:
    return (rad + np.pi) % (2 * np.pi) - np.pi

def filter_waypoints(location: np.ndarray, current_idx: int, waypoints: List[roar_py_interface.RoarPyWaypoint], threshold: float = 3.0) -> int:
    def dist_to_waypoint(waypoint: roar_py_interface.RoarPyWaypoint) -> float:
        return np.linalg.norm(location[:2] - waypoint.location[:2])
    
    for i in range(current_idx, len(waypoints) + current_idx):
        if dist_to_waypoint(waypoints[i % len(waypoints)]) < threshold:
            return i % len(waypoints)
    return current_idx

class RoarCompetitionSolution:
    def __init__(
        self,
        maneuverable_waypoints: List[roar_py_interface.RoarPyWaypoint],
        vehicle: roar_py_interface.RoarPyActor,
        camera_sensor: roar_py_interface.RoarPyCameraSensor = None,
        location_sensor: roar_py_interface.RoarPyLocationInWorldSensor = None,
        velocity_sensor: roar_py_interface.RoarPyVelocimeterSensor = None,
        rpy_sensor: roar_py_interface.RoarPyRollPitchYawSensor = None,
        occupancy_map_sensor: roar_py_interface.RoarPyOccupancyMapSensor = None,
        collision_sensor: roar_py_interface.RoarPyCollisionSensor = None,
    ) -> None:
        self.maneuverable_waypoints = maneuverable_waypoints
        self.vehicle = vehicle
        self.camera_sensor = camera_sensor
        self.location_sensor = location_sensor
        self.velocity_sensor = velocity_sensor
        self.rpy_sensor = rpy_sensor
        self.occupancy_map_sensor = occupancy_map_sensor
        self.collision_sensor = collision_sensor

    async def initialize(self) -> None:
        # Initial computation if needed
        vehicle_location = self.location_sensor.get_last_gym_observation()
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            10,
            self.maneuverable_waypoints
        )

    async def step(self) -> None:
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()
        vehicle_velocity_norm = np.linalg.norm(vehicle_velocity)

        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )

        # Calculate lookahead distances based on speed
        lookahead_distances = [
            0.44, 0.48, 0.56, 0.68, 0.90, 1.25
        ]
        waypoints_to_follow = [
            self.maneuverable_waypoints[(self.current_waypoint_idx + int(np.floor(ld * vehicle_velocity_norm))) % len(self.maneuverable_waypoints)]
            for ld in lookahead_distances
        ]

        # Determine which waypoint to follow based on current index
        if (self.current_waypoint_idx % 2775 > 2500):
            vector_to_waypoint = (waypoints_to_follow[0].location - vehicle_location)[:2]
        elif (self.current_waypoint_idx % 2775 > 1375 and self.current_waypoint_idx % 2775 < 1400):
            vector_to_waypoint = (waypoints_to_follow[3].location - vehicle_location)[:2]
        elif (self.current_waypoint_idx % 2775 > 1700 and self.current_waypoint_idx % 2775 < 2200):
            vector_to_waypoint = (waypoints_to_follow[2].location - vehicle_location)[:2]
        elif (self.current_waypoint_idx % 2773 > 1400 and self.current_waypoint_idx % 2773 < 1450):
            vector_to_waypoint = (waypoints_to_follow[0].location - vehicle_location)[:2]
        elif (self.current_waypoint_idx % 2773 > 350 and self.current_waypoint_idx % 2773 < 600):
            vector_to_waypoint = (waypoints_to_follow[0].location - vehicle_location)[:2]
        elif (self.current_waypoint_idx % 2775 > 0 and self.current_waypoint_idx % 2775 < 339):
            vector_to_waypoint = (waypoints_to_follow[3].location - vehicle_location)[:2]
        else:
            vector_to_waypoint = (waypoints_to_follow[1].location - vehicle_location)[:2]

        heading_to_waypoint = np.arctan2(vector_to_waypoint[1], vector_to_waypoint[0])
        delta_heading = normalize_rad(heading_to_waypoint - vehicle_rotation[2])

        vector_to_superfar_waypoint = (waypoints_to_follow[5].location - vehicle_location)[:2]
        superfar_turning_radius = 1.0 / np.cross([vector_to_superfar_waypoint[0], vector_to_superfar_waypoint[1], 0], [np.cos(vehicle_rotation[2]), np.sin(vehicle_rotation[2]), 0])[2] if np.linalg.norm(vector_to_superfar_waypoint) > 0 else 0.0
        superfar_curvature = 1.0 / superfar_turning_radius if superfar_turning_radius != 0 else 0.0

        steer_control = (
            -18.0 / np.sqrt(vehicle_velocity_norm) * delta_heading / np.pi
        ) if vehicle_velocity_norm > 1e-2 else -np.sign(delta_heading)
        steer_control = np.clip(steer_control, -1.0, 1.0)

        throttle_control = self.calculate_speed_control(superfar_curvature)
        control = {
            "throttle": np.clip(throttle_control, 0.0, 1.0),
            "steer": steer_control,
            "brake": np.clip(-throttle_control, 0.0, 1.0),
            "hand_brake": 0.0,
            "reverse": 0,
            "target_gear": 0
        }

        await self.vehicle.apply_action(control)
        return control

    def calculate_speed_control(self, superfar_curvature) -> float:
        if (self.current_waypoint_idx % 2775 > 2450 and self.current_waypoint_idx % 2775 < 2740) or (self.current_waypoint_idx % 2773 > 1300 and self.current_waypoint_idx % 2773 < 1375):
            if np.abs(superfar_curvature) > 20.0:
                return -0.185
            else:
                return 1.0
        elif (self.current_waypoint_idx % 2775 > 400) and (self.current_waypoint_idx % 2775 < 600):
            if np.abs(superfar_curvature) > 20:
                return 0.25
            else:
                return 1.0
        elif (self.current_waypoint_idx % 2773 > 1850) and (self.current_waypoint_idx % 2773 < 1950):
            if np.abs(superfar_curvature) > 20:
                return 0.60
            return 1.0
        elif (self.current_waypoint_idx % 2775 > 775) and (self.current_waypoint_idx % 2775 < 870):
            if np.abs(superfar_curvature) > 20:
                return 0.65
            return 1.0
        elif (self.current_waypoint_idx % 2775 > 600) and (self.current_waypoint_idx % 2775 < 700):
            if np.abs(superfar_curvature) > 20:
                return 0.95
            return 1.0
        else:
            return 1.0
