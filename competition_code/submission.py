"""
Competition instructions:
Please do not change anything else but fill out the to-do sections.
"""

from collections import deque
from functools import reduce
from typing import List, Tuple, Dict, Optional
import math
import numpy as np
import roar_py_interface
import roar_py_carla

#radians
def normalize_rad(rad : float):
    return (rad + np.pi) % (2 * np.pi) - np.pi

#Distance from one waypoint to another
def distance_p_to_p(p1: roar_py_interface.RoarPyWaypoint, p2: roar_py_interface.RoarPyWaypoint):
    return np.linalg.norm(p2.location[:2] - p1.location[:2])

#sort the waypoints
def filter_waypoints(location : np.ndarray, current_idx: int, waypoints : List[roar_py_interface.RoarPyWaypoint]) -> int:
    #from current location to closest waypoint
    def dist_to_waypoint(waypoint : roar_py_interface.RoarPyWaypoint):
        return np.linalg.norm(
            location[:2] - waypoint.location[:2]
        )
    #Loop to find closest waypoint
    for i in range(current_idx, len(waypoints) + current_idx):
        if dist_to_waypoint(waypoints[i%len(waypoints)]) < 3:
            return i % len(waypoints)
    return current_idx


#create the new waypoint
def new_x_y(x, y):
        new_location = np.array([x, y, 0])
        return roar_py_interface.RoarPyWaypoint(location=new_location, 
                                                roll_pitch_yaw=np.array([0,0,0]), 
                                                lane_width=5)

class RoarCompetitionSolution:
    def __init__(
        self,
        maneuverable_waypoints: List[roar_py_interface.RoarPyWaypoint],
        vehicle : roar_py_interface.RoarPyActor,    
        camera_sensor : roar_py_interface.RoarPyCameraSensor = None,
        location_sensor : roar_py_interface.RoarPyLocationInWorldSensor = None,
        velocity_sensor : roar_py_interface.RoarPyVelocimeterSensor = None,
        rpy_sensor : roar_py_interface.RoarPyRollPitchYawSensor = None,
        occupancy_map_sensor : roar_py_interface.RoarPyOccupancyMapSensor = None,
        collision_sensor : roar_py_interface.RoarPyCollisionSensor = None,
    ) -> None:
        # self.maneuverable_waypoints = maneuverable_waypoints[:1962] + maneuverable_waypoints[1967:]
        # startInd = 1953
        startInd_8 = 1800
        endInd_8 = 2006
        startInd_12 = 2586
        # endInd = 1967
        # endInd = startInd+len(NEW_WAYPOINTS)
        # self.maneuverable_waypoints = \
        #     maneuverable_waypoints[:startInd_8] + SEC_8_WAYPOINTS \
        #         + maneuverable_waypoints[endInd_8:] 
        self.maneuverable_waypoints = \
            SEC_1_WAYPOINTS \
            + maneuverable_waypoints[199:startInd_8] + SEC_8_WAYPOINTS \
                + maneuverable_waypoints[endInd_8:startInd_12] \
                + SEC_12_WAYPOINTS
        # self.maneuverable_waypoints = self.modified_points(maneuverable_waypoints)
        
        self.vehicle = vehicle
        self.camera_sensor = camera_sensor
        self.location_sensor = location_sensor
        self.velocity_sensor = velocity_sensor
        self.rpy_sensor = rpy_sensor
        self.occupancy_map_sensor = occupancy_map_sensor
        self.collision_sensor = collision_sensor
        self.lat_pid_controller = LatPIDController(config=self.get_lateral_pid_config())
        self.throttle_controller = ThrottleController()
        self.section_indeces = []
        self.num_ticks = 0
        self.section_start_ticks = 0
        self.current_section = -1

    async def initialize(self) -> None:
        num_sections = 12
        #indexes_per_section = len(self.maneuverable_waypoints) // num_sections
        #self.section_indeces = [indexes_per_section * i for i in range(0, num_sections)]
        # self.section_indeces = [198, 438, 547, 691, 803, 884, 1287, 1508, 1854, 1968, 2264, 2662, 2770]
        self.section_indeces = [198, 438, 547, 691, 803, 884, 1287, 1508, 1854, 1968, 2264, 2592, 2770]
        print(f"1 lap length: {len(self.maneuverable_waypoints)}")
        print(f"indexes: {self.section_indeces}")
        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()

        self.current_waypoint_idx = 10
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )

    # def modified_points(self, waypoints):
    #     new_points = []
    #     for ind, waypoint in enumerate(waypoints):
    #         if ind == 1964:
    #             new_points.append(self.new_x(waypoint, -151))
    #         elif ind == 1965:
    #             new_points.append(self.new_x(waypoint, -153))
    #         elif ind == 1966:
    #             new_points.append(self.new_x(waypoint, -155))
    #         else:
    #             new_points.append(waypoint)
    #     return new_points
        
    # def modified_points_bad(self, waypoints):
    #     end_ind = 1964
    #     num_points = 50
    #     start_ind = end_ind - num_points
    #     shift_vector = np.array([0.5, 0, 0])
    #     step_vector = shift_vector / num_points

    #     s2 = 1965
    #     num_points2 = 150
    #     shift_vector2 = np.array([0, 2.0, 0])


    #     new_points = []
    #     for ind, waypoint in enumerate(waypoints):
    #         p = waypoint
    #         if ind >= start_ind and ind < end_ind:
    #             p = self.point_plus_vec(p, step_vector * (ind - start_ind))
    #         if ind >= s2 and ind < s2 + num_points2:
    #              p = self.point_plus_vec(p, shift_vector2)
    #         new_points.append(p)
    #     return new_points

    # def modified_points_good(self, waypoints):
    #     start_ind = 1920
    #     num_points = 100
    #     end_ind = start_ind + num_points
    #     shift_vector = np.array([2.8, 0, 0])
    #     step_vector = shift_vector / num_points

    #     s2 = 1965
    #     num_points2 = 150
    #     shift_vector2 = np.array([0, 3.5, 0])

    #     s3 = 1920
    #     num_points3 = 195
    #     shift_vector3 = np.array([0.0, 0, 0])

    #     new_points = []
    #     for ind, waypoint in enumerate(waypoints):
    #         p = waypoint
    #         if ind >= start_ind and ind < end_ind:
    #             p = self.point_plus_vec(p, step_vector * (end_ind - ind))
    #             # p = self.point_plus_vec(p, step_vector * (end_ind - ind))
    #         if ind >= s2 and ind < s2 + num_points2:
    #             p = self.point_plus_vec(p, shift_vector2)
    #         if ind >= s3 and ind < s3 + num_points3:
    #             p = self.point_plus_vec(p, shift_vector3)
    #         new_points.append(p)
    #     return new_points

    def point_plus_vec(self, waypoint, vector):
        new_location = waypoint.location + vector
        # new_location = np.array([waypoint.location[0], new_y, waypoint.location[2]])
        return roar_py_interface.RoarPyWaypoint(location=new_location,
                                                roll_pitch_yaw=waypoint.roll_pitch_yaw,
                                                lane_width=waypoint.lane_width)


    # def modified_points_also_bad(self, waypoints):
    #     new_points = []
    #     for ind, waypoint in enumerate(waypoints):
    #         if ind >= 1962 and ind <= 2027:
    #             new_points.append(self.new_point(waypoint, self.new_y(waypoint.location[0])))
    #         else:
    #             new_points.append(waypoint)
    #     return new_points
    

    # def new_x(self, waypoint, new_x):
    #     new_location = np.array([new_x, waypoint.location[1], waypoint.location[2]])
    #     return roar_py_interface.RoarPyWaypoint(location=new_location, 
    #                                             roll_pitch_yaw=waypoint.roll_pitch_yaw, 
    #                                             lane_width=waypoint.lane_width)
    # def new_point(self, waypoint, new_y):
    #     new_location = np.array([waypoint.location[0], new_y, waypoint.location[2]])
    #     return roar_py_interface.RoarPyWaypoint(location=new_location, 
    #                                             roll_pitch_yaw=waypoint.roll_pitch_yaw, 
    #                                             lane_width=waypoint.lane_width)
    # def new_y(self, x):

    #     y = -math.sqrt(102**2 - (x + 210)**2) - 962
    #     #print(str(x) + ',' + str(y))
    #     return y
        
        # a=0.000322627
        # b=2.73377
        # y = a * ( (abs(x + 206))**b ) - 1063.5
        # return y

    async def step(
        self
    ) -> None:
        """
        This function is called every world step.
        Note: You should not call receive_observation() on any sensor here, instead use get_last_observation() to get the last received observation.
        You can do whatever you want here, including apply_action() to the vehicle.
        """
        self.num_ticks += 1

        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()
        vehicle_velocity_norm = np.linalg.norm(vehicle_velocity)
        current_speed_kmh = vehicle_velocity_norm * 3.6
        
        # Find the waypoint closest to the vehicle
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )

        # compute and print section timing
        for i, section_ind in enumerate(self.section_indeces):
            if section_ind -2 <= self.current_waypoint_idx \
                and self.current_waypoint_idx <= section_ind + 2 \
                    and i != self.current_section:
                elapsed_ticks = self.num_ticks - self.section_start_ticks
                self.section_start_ticks = self.num_ticks
                self.current_section = i
                print(f"Section {i}: {elapsed_ticks}")

        new_waypoint_index = self.get_lookahead_index(current_speed_kmh)
        waypoint_to_follow = self.next_waypoint_smooth(current_speed_kmh)
        #waypoint_to_follow = self.maneuverable_waypoints[new_waypoint_index]

        # Proportional controller to steer the vehicle
        steer_control = self.lat_pid_controller.run(
            vehicle_location, vehicle_rotation, current_speed_kmh, self.current_section, waypoint_to_follow)

        # Proportional controller to control the vehicle's speed
        waypoints_for_throttle = \
            (self.maneuverable_waypoints + self.maneuverable_waypoints)[new_waypoint_index:new_waypoint_index + 300]
        throttle, brake, gear = self.throttle_controller.run(
            self.current_waypoint_idx, waypoints_for_throttle, vehicle_location, current_speed_kmh, self.current_section)

        control = {
            "throttle": np.clip(throttle, 0.0, 1.0),
            "steer": np.clip(steer_control, -1.0, 1.0),
            "brake": np.clip(brake, 0.0, 1.0),
            "hand_brake": 0.0,
            "reverse": 0,
            "target_gear": gear
        }




        await self.vehicle.apply_action(control)
        return control

    def get_lookahead_value(self, speed):
        speed_to_lookahead_dict = {
            70: 12,
            90: 12,
            110: 13,
            130: 14,
            160: 16,
            180: 20,
            200: 24,
            300: 24
        }
        num_waypoints = 3
        for speed_upper_bound, num_points in speed_to_lookahead_dict.items():
            if speed < speed_upper_bound:
              num_waypoints = num_points
              break
            
            if self.current_section in [6,7]:
                num_waypoints = num_points*3//2

        return num_waypoints

    def get_lookahead_index(self, speed):
        num_waypoints = self.get_lookahead_value(speed)
        # print("speed " + str(speed) 
        #       + " cur_ind " + str(self.current_waypoint_idx) 
        #       + " num_points " + str(num_waypoints) 
        #       + " index " + str((self.current_waypoint_idx + num_waypoints) % len(self.maneuverable_waypoints)) )
        return (self.current_waypoint_idx + num_waypoints) % len(self.maneuverable_waypoints)
    
    def get_lateral_pid_config(self):
        conf = {
            "30": {
                "Kp": 1.00,
                "Kd": 0.03,
                "Ki": 0.02
            },
            "40": {
                "Kp": 0.90,
                "Kd": 0.04,
                "Ki": 0.03
            },
            "50": {
                "Kp": 0.85,
                "Kd": 0.05,
                "Ki": 0.04
            },
            "60": {
                "Kp": 0.8034,
                "Kd": 0.0521,
                "Ki": 0.0518
            },
            "70": {
                "Kp": 0.7025,
                "Kd": 0.0698,
                "Ki": 0.0712
            },
            "80": {
                "Kp": 0.6621,
                "Kd": 0.0807,
                "Ki": 0.0804
            },
            "90": {
                "Kp": 0.6289,
                "Kd": 0.0903,
                "Ki": 0.0921
            },
            "100": {
                "Kp": 0.6012,
                "Kd": 0.1005,
                "Ki": 0.1013
            },
            "120": {
                "Kp": 0.5243,
                "Kd": 0.0987,
                "Ki": 0.1004
            },
            "130": {
                "Kp": 0.5114,
                "Kd": 0.1012,
                "Ki": 0.0905
            },
            "140": {
                "Kp": 0.4489,
                "Kd": 0.0995,
                "Ki": 0.0899
            },
            "160": {
                "Kp": 0.3987,
                "Kd": 0.0502,
                "Ki": 0.0589
            },
            "180": {
                "Kp": 0.2874,
                "Kd": 0.0221,
                "Ki": 0.0517
            },
            "200": {
                "Kp": 0.2876,
                "Kd": 0.0312,
                "Ki": 0.0421
            },
            "230": {
                "Kp": 0.2658,
                "Kd": 0.0401,
                "Ki": 0.0504
            },
            "250": {
                "Kp": 0.2500,
                "Kd": 0.0350,
                "Ki": 0.0450
            },
            "270": {
                "Kp": 0.2450,
                "Kd": 0.0320,
                "Ki": 0.0400
            },
            "300": {
                "Kp": 0.2065,
                "Kd": 0.0082,
                "Ki": 0.0181
            }
        }
        return conf



    # The idea and code for averaging points is from smooth_waypoint_following_local_planner.py
    def next_waypoint_smooth(self, current_speed: float):
        if current_speed > 70 and current_speed < 300:
            target_waypoint = self.average_point(current_speed)
        else:
            new_waypoint_index = self.get_lookahead_index(current_speed)
            target_waypoint = self.maneuverable_waypoints[new_waypoint_index]
        return target_waypoint

    def average_point(self, current_speed):
        next_waypoint_index = self.get_lookahead_index(current_speed)
        lookahead_value = self.get_lookahead_value(current_speed)
        num_points = lookahead_value * 2
        
        # if self.current_section in [12]:
        #     num_points = lookahead_value
        if self.current_section in [8,9]:
            # num_points = lookahead_value // 2
            num_points = lookahead_value * 2
            # num_points = lookahead_value
            # num_points = 1
        start_index_for_avg = (next_waypoint_index - (num_points // 2)) % len(self.maneuverable_waypoints)

        next_waypoint = self.maneuverable_waypoints[next_waypoint_index]
        next_location = next_waypoint.location
  
        sample_points = [(start_index_for_avg + i) % len(self.maneuverable_waypoints) for i in range(0, num_points)]
        if num_points > 3:
            location_sum = reduce(lambda x, y: x + y,
                                  (self.maneuverable_waypoints[i].location for i in sample_points))
            num_points = len(sample_points)
            new_location = location_sum / num_points
            shift_distance = np.linalg.norm(next_location - new_location)
            max_shift_distance = 2.0
            if self.current_section in [1,2]:
                max_shift_distance = 0.2
            if self.current_section in [6, 7]:
                max_shift_distance = 1.0
            if self.current_section in [8,9]:
                max_shift_distance = 2.8
            if self.current_section in [10,11]:
                max_shift_distance = 0.2
            if self.current_section in [12]:
                max_shift_distance = 0.4
            if shift_distance > max_shift_distance:
                uv = (new_location - next_location) / shift_distance
                new_location = next_location + uv*max_shift_distance

            target_waypoint = roar_py_interface.RoarPyWaypoint(location=new_location, 
                                                               roll_pitch_yaw=np.ndarray([0, 0, 0]), 
                                                               lane_width=0.0)
            # if next_waypoint_index > 1900 and next_waypoint_index < 2300:
            #   print("AVG: next_ind:" + str(next_waypoint_index) + " next_loc: " + str(next_location) 
            #       + " new_loc: " + str(new_location) + " shift:" + str(shift_distance)
            #       + " num_points: " + str(num_points) + " start_ind:" + str(start_index_for_avg)
            #       + " curr_speed: " + str(current_speed))

        else:
            target_waypoint =  self.maneuverable_waypoints[next_waypoint_index]

        return target_waypoint

class LatPIDController():
    def __init__(self, config: dict, dt: float = 0.05):
        self.config = config
        self.steering_boundary = (-1.0, 1.0)
        self._error_buffer = deque(maxlen=10)
        self._dt = dt

    def run(self, vehicle_location, vehicle_rotation, current_speed, cur_section, next_waypoint) -> float:
        """
        Calculates a vector that represent where you are going.
        Args:
            next_waypoint ():
            **kwargs ():

        Returns:
            lat_control
        """
        # calculate a vector that represent where you are going
        v_begin = vehicle_location
        direction_vector = np.array([
            np.cos(normalize_rad(vehicle_rotation[2])),
            np.sin(normalize_rad(vehicle_rotation[2])),
            0])
        #find the end of the vector
        v_end = v_begin + direction_vector
        #finds the actual vector
        v_vec = np.array([(v_end[0] - v_begin[0]), (v_end[1] - v_begin[1]), 0])
        
        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location[0] - v_begin[0],
                next_waypoint.location[1] - v_begin[1],
                0,
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        _cross = np.cross(v_vec_normed, w_vec_normed)

        if _cross[2] > 0:
            error *= -1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d, k_i = self.find_k_values(cur_section, current_speed=current_speed, config=self.config)

        lat_control = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )
        
        # PIDFastController.sdprint("steer: " + str(lat_control) + " err" + str(error) + " k_p=" + str(k_p) + " de" + str(_de) + " k_d=" + str(k_d) 
        #     + " ie" + str(_ie) + " k_i=" + str(k_i) + " sum" + str(sum(self._error_buffer)))
        # print("cross " + str(_cross))
        # print("loc " + str(vehicle_location) + " rot "  + str(vehicle_rotation))
        # print(" next.loc " + str(next_waypoint.location))

        # print("steer: " + str(lat_control) + " speed: " + str(current_speed) + " err" + str(error) + " k_p=" + str(k_p) + " de" + str(_de) + " k_d=" + str(k_d) 
        #     + " ie" + str(_ie) + " k_i=" + str(k_i) + " sum" + str(sum(self._error_buffer)))
        # print("   err P " + str(k_p * error) + " D " + str(k_d * _de) + " I " + str(k_i * _ie))

        return lat_control
    
    def find_k_values(self, cur_section, current_speed: float, config: dict) -> np.array:
        k_p, k_d, k_i = 1, 0, 0
        #for sections 8,9,10,11, predefined PID values are set
        if cur_section in [8, 9, 10, 11]:
        #   return np.array([0.3, 0.1, 0.25]) # ok for mu=1.2
        #   return np.array([0.2, 0.03, 0.15])
        #   return np.array([0.3, 0.06, 0.03]) # ok for mu=1.8
        #   return np.array([0.42, 0.05, 0.02]) # ok for mu=2.0
        #   return np.array([0.45, 0.05, 0.02]) # ok for mu=2.2
          return np.array([0.58, 0.05, 0.02]) # 
        # if cur_section in [12]:
        #   return np.array([0.4, 0.05, 0.02]) # 
        #iterates through config, each item maps a speed upper bound to a set of PID rules
        for speed_upper_bound, kvalues in config.items():
            #converts speed_upper_bound to a float
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])

    
    def normalize_rad(rad : float):
        return (rad + np.pi) % (2 * np.pi) - np.pi

class SpeedData:
    def __init__(self, distance_to_section, current_speed, target_speed, recommended_speed):
        self.current_speed = current_speed
        self.distance_to_section = distance_to_section
        self.target_speed_at_distance = target_speed
        self.recommended_speed_now = recommended_speed
        self.speed_diff = current_speed - recommended_speed

class ThrottleController():
    #Debug 
    def __init__(self):
        self.max_radius = 10000
        self.max_speed = 300
        self.intended_target_distance = [0, 30, 60, 90, 120, 150, 180]
        self.target_distance = [0, 30, 60, 90, 120, 150, 180]
        self.close_index = 0
        self.mid_index = 1
        self.far_index = 2
        self.tick_counter = 0
        self.previous_speed = 1.0
        self.brake_ticks = 0

        # for testing how fast the car stops
        self.brake_test_counter = 0
        self.brake_test_in_progress = False

        # for s in self.__class__.debug_strings:
        #     print(s)

    def run(self, cur_wp_index, waypoints, current_location, current_speed, current_section) -> (float, float, int):
        self.tick_counter += 1   #keeps track the number of updates
        #sets throttle and break values 
        throttle, brake = self.get_throttle_and_brake(cur_wp_index, current_location, current_speed, current_section, waypoints)
        #sets gear
        gear = max(1, (int)(current_speed / 60))
        if throttle == -1:
            gear = -1

        # self.dprint("--- " + str(throttle) + " " + str(brake) 
        #             + " steer " + str(steering)
        #             + "     loc x,z" + str(self.agent.vehicle.transform.location.x)
        #             + " " + str(self.agent.vehicle.transform.location.z)) 
        #updates speed for future comparisons
        self.previous_speed = current_speed
        #if brakes are applied and there are remaining ticks, making sure that it is breaking long enough
        if self.brake_ticks > 0 and brake > 0:
            self.brake_ticks -= 1

        # throttle = 0.05 * (100 - current_speed)
        return throttle, brake, gear
    # determines throttle and break values based on curr speed and layout of the track
    def get_throttle_and_brake(self, cur_wp_index, current_location, current_speed, current_section, waypoints):
        #gets a set of waypoints based on curr location
        wp = self.get_next_interesting_waypoints(current_location, waypoints)
        #calculates the curvature 
        r1 = self.get_radius(wp[self.close_index : self.close_index + 3])
        r2 = self.get_radius(wp[self.mid_index : self.mid_index + 3])
        r3 = self.get_radius(wp[self.far_index : self.far_index + 3])
        #computes target speed for each section 
        target_speed1 = self.get_target_speed(r1, current_section, current_speed)
        target_speed2 = self.get_target_speed(r2, current_section, current_speed)
        target_speed3 = self.get_target_speed(r3, current_section, current_speed)
        #calculates distances and target speeds for different points
        close_distance = self.target_distance[self.close_index] + 3
        mid_distance = self.target_distance[self.mid_index]
        far_distance = self.target_distance[self.far_index]
        speed_data = []
        speed_data.append(self.speed_for_turn(close_distance, target_speed1, current_speed))
        speed_data.append(self.speed_for_turn(mid_distance, target_speed2, current_speed))
        speed_data.append(self.speed_for_turn(far_distance, target_speed3, current_speed))
        #calculates additional target speed for a wider turn (larger segment of waypoints)
        if current_speed > 100:
            # at high speed use larger spacing between points to look further ahead and detect wide turns.
            r4 = self.get_radius([wp[self.close_index], wp[self.close_index+3], wp[self.close_index+6]])
            target_speed4 = self.get_target_speed(r4, current_section, current_speed)
            speed_data.append(self.speed_for_turn(close_distance, target_speed4, current_speed))
        #selects optimal speed  
        update = self.select_speed(speed_data)
        
        # self.print_speed(" -- SPEED: ", 
        #                  speed_data[0].recommended_speed_now, 
        #                  speed_data[1].recommended_speed_now, 
        #                  speed_data[2].recommended_speed_now,
        #                  (0 if len(speed_data) < 4 else speed_data[3].recommended_speed_now), 
        #                  current_speed)
        #determines the throttle and break values 
        t, b = self.speed_data_to_throttle_and_brake(update)
        #debugging print
        # self.dprint("--- (" + str(cur_wp_index) + ") throt " + str(t) + " brake " + str(b) + "---")
        return t, b
    
    # gets throttle and brake values based on current speed data, adjuests speed accordingly

    def speed_data_to_throttle_and_brake(self, speed_data: SpeedData):
        percent_of_max = speed_data.current_speed / speed_data.recommended_speed_now

        # self.dprint("dist=" + str(round(speed_data.distance_to_section)) + " cs=" + str(round(speed_data.current_speed, 2)) 
        #             + " ts= " + str(round(speed_data.target_speed_at_distance, 2)) 
        #             + " maxs= " + str(round(speed_data.recommended_speed_now, 2)) + " pcnt= " + str(round(percent_of_max, 2)))
        # calculates ratio of curr_speed to recommended speed
        percent_change_per_tick = 0.07 # speed drop for one time-tick of braking
        speed_up_threshold = 0.99  #deciding when to speed up
        throttle_decrease_multiple = 0.7  
        throttle_increase_multiple = 1.25
        percent_speed_change = (speed_data.current_speed - self.previous_speed) / (self.previous_speed + 0.0001) # avoid division by zero

        if percent_of_max > 1:   # curr speed > recommended speed, function considers breaking
            # Consider slowing down
            #checks if it should break based on the difference between perent_of_max and percent-change_per_tick
            #if speed is dropping fast, it reduces breaking and may apply throttle
            brake_threshold_multiplier = 1.0
            if speed_data.current_speed > 200:
                brake_threshold_multiplier = 1.0
            if percent_of_max > 1 + (brake_threshold_multiplier * percent_change_per_tick):
                if self.brake_ticks > 0:
                    # self.dprint("tb: tick" + str(self.tick_counter) + " brake: counter" + str(self.brake_ticks))
                    return -1, 1
                # if speed is not decreasing fast, hit the brake.
                if self.brake_ticks <= 0 and not self.speed_dropping_fast(percent_change_per_tick, speed_data.current_speed):
                    # start braking, and set for how many ticks to brake
                    self.brake_ticks = math.floor((percent_of_max - 1) / percent_change_per_tick)
                    # TODO: try 
                    # self.brake_ticks = 1, or (1 or 2 but not more)
                    # self.dprint("tb: tick" + str(self.tick_counter) + " brake: initiate counter" + str(self.brake_ticks))
                    return -1, 1
                else:
                    # speed is already dropping fast, ok to throttle because the effect of throttle is delayed
                    # self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle early1: sp_ch=" + str(percent_speed_change))
                    self.brake_ticks = 0 # done slowing down. clear brake_ticks
                    return 1, 0
            else:
                #if speed is dropping rapidly it applies full throttle
                # if speed is below speed_up_threshold, applies full throttle
                #otherwise, it calculates the throttle needed to maintain the curr speed and adjusts accordingly
                if self.speed_dropping_fast(percent_change_per_tick, speed_data.current_speed):
                    # speed is already dropping fast, ok to throttle because the effect of throttle is delayed
                    # self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle early2: sp_ch=" + str(percent_speed_change))
                    self.brake_ticks = 0 # done slowing down. clear brake_ticks
                    return 1, 0
                throttle_to_maintain = self.get_throttle_to_maintain_speed(speed_data.current_speed)
                if percent_of_max > 1.02 or percent_speed_change > (-percent_change_per_tick / 2):
                    # self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle down: sp_ch=" + str(percent_speed_change))
                    return throttle_to_maintain * throttle_decrease_multiple, 0 # coast, to slow down
                else:
                    # self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle maintain: sp_ch=" + str(percent_speed_change))
                    return throttle_to_maintain, 0
        else:
            self.brake_ticks = 0 # done slowing down. clear brake_ticks
            # Consider speeding up
            if self.speed_dropping_fast(percent_change_per_tick, speed_data.current_speed):
                # speed is dropping fast, ok to throttle because the effect of throttle is delayed
                # self.dprint("tb: tick" + str(self.tick_counter) + " throttle: full speed drop: sp_ch=" + str(percent_speed_change))
                return 1, 0
            if percent_of_max < speed_up_threshold:
                # self.dprint("tb: tick" + str(self.tick_counter) + " throttle full: p_max=" + str(percent_of_max))
                return 1, 0
            throttle_to_maintain = self.get_throttle_to_maintain_speed(speed_data.current_speed)
            if percent_of_max < 0.98 or percent_speed_change < -0.01:
                # self.dprint("tb: tick" + str(self.tick_counter) + " throttle up: sp_ch=" + str(percent_speed_change))
                return throttle_to_maintain * throttle_increase_multiple, 0 
            else:
                # self.dprint("tb: tick" + str(self.tick_counter) + " throttle maintain: sp_ch=" + str(percent_speed_change))
                return throttle_to_maintain, 0

    # used to detect when speed is dropping due to brakes applied earlier. speed delta has a steep negative slope.
    def speed_dropping_fast(self, percent_change_per_tick: float, current_speed):
        percent_speed_change = (current_speed - self.previous_speed) / (self.previous_speed + 0.0001) # avoid division by zero
        return percent_speed_change < (-percent_change_per_tick / 2)
    #could improve
    # find speed_data with smallest recommended speed
    def select_speed(self, speed_data: [SpeedData]):
        min_speed = 1000  #uses high value as comparison to make sure that actual speeds will be lower
        index_of_min_speed = -1
        #finding minimum speed 
        for i, sd in enumerate(speed_data):
            if sd.recommended_speed_now < min_speed:
                min_speed = sd.recommended_speed_now
                index_of_min_speed = i

        if index_of_min_speed != -1:
            return speed_data[index_of_min_speed]
        else:
            return speed_data[0]
        
    # calculate throttle value to maintain given speed
    def get_throttle_to_maintain_speed(self, current_speed: float):
        throttle = 0.6 + current_speed/1000
        return throttle

        #calculates max achievable speed for vehicle in given distance 

    def speed_for_turn(self, distance: float, target_speed: float, current_speed: float):
        d = (1/675) * (target_speed**2) + distance
        max_speed = math.sqrt(675 * d)
        return SpeedData(distance, current_speed, target_speed, max_speed)


    # calculates max speed a vehicle can achieve after traveling specified distance (break, acceleration)
    def speed_for_turn_fix_physics(self, distance: float, target_speed: float, current_speed: float):
        # fix physics
        braking_decceleration = 66.0 # try 11, 14, 56
        #sets max speed for turning
        max_speed = math.sqrt((target_speed**2) + 2 * distance * (braking_decceleration + 9.81))
        return SpeedData(distance, current_speed, target_speed, max_speed)

    def get_next_interesting_waypoints(self, current_location, more_waypoints):
        # return a list of points with distances approximately as given 
        # in intended_target_distance[] from the current location.
        points = [] # store waypoints
        dist = [] # for debugging
        #get the starting waypoint
        start = roar_py_interface.RoarPyWaypoint(current_location, np.ndarray([0, 0, 0]), 0.0)
        # start = self.agent.vehicle.transform
        points.append(start)  
        curr_dist = 0  # keeps track of total distancec
        num_points = 0   #keeps track of waypoints
        for p in more_waypoints:  #go through the waypoints
            end = p   #set end to current waypoint
            num_points += 1  #increment number of waypoints passed
            # print("start " + str(start) + "\n- - - - -\n")
            # print("end " + str(end) +     "\n- - - - -\n")
            curr_dist += distance_p_to_p(start, end)  #calculate distance from start to end and add to currdistance
            # curr_dist += start.location.distance(end.location)
            # if curr distance is > the intended target distance for next point
            if curr_dist > self.intended_target_distance[len(points)]:
                self.target_distance[len(points)] = curr_dist  #update target distance
                points.append(end)  #store waypoint end
                dist.append(curr_dist)  #add distance 
            start = end  #update the new start
            if len(points) >= len(self.target_distance):   #if the number of points is > the intended target distance, break
                break

        #self.dprint("wp dist " +  str(dist))
        return points # return waypoints
    #returns radius of the curve
    def get_radius(self, wp):
        #gets the points of the 3 closest waypoints
        point1 = (wp[0].location[0], wp[0].location[1])
        point2 = (wp[1].location[0], wp[1].location[1])
        point3 = (wp[2].location[0], wp[2].location[1])

        # Calculating length of all three sides
        len_side_1 = round( math.dist(point1, point2), 3)
        len_side_2 = round( math.dist(point2, point3), 3)
        len_side_3 = round( math.dist(point1, point3), 3)
        #If one of the sides is < small_num, the points are close together (degenerate triangle) so it returns max radius
        small_num = 0.01
        if len_side_1 < small_num or len_side_2 < small_num or len_side_3 < small_num:
            return self.max_radius

        # sp is semi-perimeter 
        sp = (len_side_1 + len_side_2 + len_side_3) / 2

        # Calculating area using Herons formula
        area_squared = sp * (sp - len_side_1) * (sp - len_side_2) * (sp - len_side_3)
        #if the area is less than small_num that means that the turn is really small (collinear points)
        if area_squared < small_num:
            return self.max_radius
        # Calculating curvature using Menger curvature formula
        radius = (len_side_1 * len_side_2 * len_side_3) / (4 * math.sqrt(area_squared))
        return radius
    


    def get_target_speed(self, radius: float, current_section, current_speed):
        # if the radius of the curve is greater than the max radius, go full speed
        if radius >= self.max_radius:
            return self.max_speed
        #self.section_indeces = [198, 438, 547, 691, 803, 884, 1287, 1508, 1854, 1968, 2264, 2662, 2770]
        #old section indeces = [0, 277, 554, 831, 1108, 1662, 1939, 2216, 2493]
        # sets the frictino components for each section
        mu = 2.5
        if current_section == 0:
            mu = 1
        if current_section == 1:
            mu = 2.0
        if current_section == 2:
            mu = 1.95
        if current_section == 3:
            mu = 1
        if current_section == 4:
            mu = 3.25
        if current_section == 5:
            mu = 3.4
        if current_section == 6:
            mu = 1.95
        if current_section == 7:
            mu = 0.5
        # if current_section == 7 and current_speed<150:
        #     mu = 1.8
        if current_section == 8:
            mu = 3.7
        if current_section == 9:
            mu = 3.6
        if current_section == 10:
            mu = 3.8
        if current_section == 11:
            mu = 1.9
        if current_section == 12:
            mu = 1.9
        '''old friction coefficients (goes with old sections): 
        if current_section == 6:
            mu = 1.1
        if current_section == 7:
            mu = 1.5
        if current_section == 9:
            mu = 1.5'''
        # sets the target speed using formula. sqrt(friction * gravity(9.81) * radius of curve ) * unit conversion (3.6)
        target_speed = math.sqrt(mu*9.81*radius) * 3.6
        #makes sure that the target speed is between 20 - maxspeed so it doesnt go too fast
        return max(20, min(target_speed, self.max_speed))  # clamp between 20 and max_speed


    # def print_speed(self, text: str, s1: float, s2: float, s3: float, s4: float, curr_s: float):
    #     self.dprint(text + " s1= " + str(round(s1, 2)) + " s2= " + str(round(s2, 2)) + " s3= " 
    #                 + str(round(s3, 2)) + " s4= " + str(round(s4, 2))
    #         + " cspeed= " + str(round(curr_s, 2)))

    # # debug print
    # def dprint(self, text):
    #     if self.display_debug:
    #         print(text)
    #         self.debug_strings.append(text)

SEC_8_WAYPOINTS = [
  new_x_y(-104.11528778076172, -726.1124877929688),
  new_x_y(-104.1638900756836, -728.1100463867188),
  new_x_y(-104.21249237060549, -730.1076049804688),
  new_x_y(-104.26109466552734, -732.1051635742188),
  new_x_y(-104.30969696044922, -734.1027221679688),
  new_x_y(-104.3582992553711, -736.100341796875),
  new_x_y(-104.40690155029296, -738.097900390625),
  new_x_y(-104.45550384521485, -740.095458984375),
  new_x_y(-104.50410614013671, -742.093017578125),
  new_x_y(-104.55270843505859, -744.090576171875),
  new_x_y(-104.60131072998048, -746.088134765625),
  new_x_y(-104.6499053955078, -748.085693359375),
  new_x_y(-104.69850769042968, -750.083251953125),
  new_x_y(-104.74710998535156, -752.0808715820312),
  new_x_y(-104.79571228027343, -754.0784301757812),
  new_x_y(-104.84431457519533, -756.0759887695312),
  new_x_y(-104.8929168701172, -758.0735473632812),
  new_x_y(-104.94151916503907, -760.0711059570312),
  new_x_y(-104.99012145996093, -762.0687255859375),
  new_x_y(-105.0387237548828, -764.0662841796875),
  new_x_y(-105.08732604980467, -766.0638427734375),
  new_x_y(-105.13592834472657, -768.0614013671875),
  new_x_y(-105.18453063964844, -770.0589599609375),
  new_x_y(-105.23313293457032, -772.0565185546875),
  new_x_y(-105.2817352294922, -774.0540771484375),
  new_x_y(-105.33033752441406, -776.0516357421875),
  new_x_y(-105.37893981933594, -778.0491943359375),
  new_x_y(-105.4275421142578, -780.0468139648438),
  new_x_y(-105.47614440917967, -782.0443725585938),
  new_x_y(-105.52474670410156, -784.0419311523438),
  new_x_y(-105.57334899902344, -786.03955078125),
  new_x_y(-105.62195129394533, -788.037109375),
  new_x_y(-105.67055358886721, -790.03466796875),
  new_x_y(-105.71915588378906, -792.0322265625),
  new_x_y(-105.76775817871093, -794.02978515625),
  new_x_y(-105.8163604736328, -796.02734375),
  new_x_y(-105.86496276855468, -798.02490234375),
  new_x_y(-105.91356506347657, -800.0224609375),
  new_x_y(-105.96216735839843, -802.02001953125),
  new_x_y(-106.01076965332032, -804.017578125),
  new_x_y(-106.0593719482422, -806.0151977539062),
  new_x_y(-106.10797424316407, -808.0127563476562),
  new_x_y(-106.15657653808594, -810.0103149414062),
  new_x_y(-106.20517883300779, -812.0079345703125),
  new_x_y(-106.25378112792967, -814.0054931640625),
  new_x_y(-106.30238342285156, -816.0030517578125),
  new_x_y(-106.35098571777344, -818.0006103515625),
  new_x_y(-106.39958801269533, -819.9981689453125),
  new_x_y(-106.4481903076172, -821.9957275390625),
  new_x_y(-106.49679260253906, -823.9932861328125),
  new_x_y(-106.54539489746094, -825.9908447265625),
  new_x_y(-106.6439971923828, -827.9884033203125),
  new_x_y(-106.74259948730467, -829.9859619140625),
  new_x_y(-106.84120178222656, -831.9835815429688),
  new_x_y(-106.93980407714844, -833.9811401367188),
  new_x_y(-107.03840637207033, -835.9786987304688),
  new_x_y(-107.13700103759766, -837.976318359375),
  new_x_y(-107.23560333251952, -839.973876953125),
  new_x_y(-107.33421325683594, -841.971435546875),
  new_x_y(-107.43280792236328, -843.968994140625),
  new_x_y(-107.53141021728516, -845.966552734375),
  new_x_y(-107.63001251220705, -847.964111328125),
  new_x_y(-107.7286148071289, -849.961669921875),
  new_x_y(-107.82721710205078, -851.959228515625),
  new_x_y(-107.92581939697266, -853.956787109375),
  new_x_y(-108.02442169189452, -855.954345703125),
  new_x_y(-108.1230239868164, -857.9519653320312),
  new_x_y(-108.22162628173828, -859.9495239257812),
  new_x_y(-108.32022857666016, -861.9470825195312),
  new_x_y(-108.41883087158205, -863.9447021484375),
  new_x_y(-108.5174331665039, -865.9422607421875),
  new_x_y(-108.61603546142578, -867.9398193359375),
  new_x_y(-108.71463775634766, -869.9373779296875),
  new_x_y(-108.81324005126952, -871.9349365234375),
  new_x_y(-108.9118423461914, -873.9324951171875),
  new_x_y(-109.01044464111328, -875.9300537109375),
  new_x_y(-109.10904693603516, -877.9276123046875),
  new_x_y(-109.20764923095705, -879.9251708984375),
  new_x_y(-109.3062515258789, -881.9227294921875),
  new_x_y(-109.40485382080078, -883.9203491210938),
  new_x_y(-109.50345611572266, -885.9179077148438),
  new_x_y(-109.60205841064452, -887.9154663085938),
  new_x_y(-109.7006607055664, -889.9130859375),
  new_x_y(-109.79926300048828, -891.91064453125),
  new_x_y(-109.89786529541016, -893.908203125),
  new_x_y(-109.99646759033205, -895.90576171875),
  new_x_y(-110.0950698852539, -897.9033203125),
  new_x_y(-110.19367218017578, -899.90087890625),
  new_x_y(-110.29227447509766, -901.8984375),
  new_x_y(-110.390869140625, -903.89599609375),
  new_x_y(-110.48947143554688, -905.8935546875),
  new_x_y(-110.58807373046876, -907.89111328125),
  new_x_y(-110.68667602539062, -909.8887329101562),
  new_x_y(-110.7852783203125, -911.8862915039062),
  new_x_y(-110.88388061523438, -913.8838500976562),
  new_x_y(-110.98248291015624, -915.8814697265624),
  new_x_y(-111.08108520507812, -917.8790283203124),
  new_x_y(-111.1796875, -919.8765869140624),
  new_x_y(-111.27828979492188, -921.8741455078124),
  new_x_y(-111.37689208984376, -923.8717041015624),
  new_x_y(-111.47549438476562, -925.8692626953124),
  new_x_y(-111.5740966796875, -927.8668212890624),
  new_x_y(-111.67269897460938, -929.8643798828124),
  new_x_y(-111.77130126953124, -931.8619384765624),
  new_x_y(-111.86990356445312, -933.8594970703124),
  new_x_y(-111.968505859375, -935.8571166992188),
  new_x_y(-112.06756591796876, -937.8546752929688),
  new_x_y(-112.17355346679688, -939.8518676757812),
  new_x_y(-112.2884521484375, -941.8485717773438),
  new_x_y(-112.4122314453125, -943.8447265625),
  new_x_y(-112.54489135742188, -945.84033203125),
  new_x_y(-112.68646240234376, -947.8353271484376),
  new_x_y(-112.8369140625, -949.8296508789062),
  new_x_y(-112.99624633789062, -951.8233032226562),
  new_x_y(-113.16445922851562, -953.8162231445312),
  new_x_y(-113.3515537600983, -955.8665279809941),
  new_x_y(-113.53996902717849, -957.9167117971767),
  new_x_y(-113.73102552986727, -959.9666510205336),
  new_x_y(-113.9260432892828, -962.0162169747531),
  new_x_y(-114.12634158750711, -964.0652733298231),
  new_x_y(-114.33323868289689, -966.1136735545402),
  new_x_y(-114.54805149255013, -968.1612583724475),
  new_x_y(-114.77209523374661, -970.2078532223375),
  new_x_y(-115.006683016202, -972.2532657246397),
  new_x_y(-115.25312537700373, -974.297283155235),
  new_x_y(-115.51272975013325, -976.3396699284953),
  new_x_y(-115.78679986252514, -978.3801650916428),
  new_x_y(-116.07663504867058, -980.4184798328554),
  new_x_y(-116.38352947584305, -982.4542950059051),
  new_x_y(-116.70877127210922, -984.4872586745246),
  new_x_y(-117.05364154939126, -986.5169836801274),
  new_x_y(-117.41941331396944, -988.5430452369803),
  new_x_y(-117.80735025695967, -990.5649785594276),
  new_x_y(-118.21870541747198, -992.5822765263035),
  new_x_y(-118.65471971135594, -994.5943873882336),
  new_x_y(-119.11662031867073, -996.6007125241229),
  new_x_y(-119.605618923285, -998.6006042537522),
  new_x_y(-120.12290979831764, -1000.5933637140547),
  new_x_y(-120.66966773147875, -1002.578238807317),
  new_x_y(-121.24704578476556, -1004.5544222302485),
  new_x_y(-121.85617288341206, -1006.5210495935763),
  new_x_y(-122.49815122949076, -1008.4771976425546),
  new_x_y(-123.17405353612175, -1010.4218825895213),
  new_x_y(-123.88492007886275, -1012.3540585703897),
  new_x_y(-124.63175556153985, -1014.2726162377194),
  new_x_y(-125.41552579453258, -1016.1763815037664),
  new_x_y(-126.23715418435646, -1018.064114447671),
  new_x_y(-127.0975180342922, -1019.9345084016783),
  new_x_y(-127.99744465679828, -1021.7861892320115),
  new_x_y(-129.29904174804688, -1024.0030168456112),
  new_x_y(-130.23733520507812, -1025.3253234392039),
  new_x_y(-131.21519470214844, -1026.6535923650738),
  new_x_y(-132.2321319580078, -1027.9847638155609),
  new_x_y(-133.28762817382812, -1029.3159715472532),
  new_x_y(-134.38116455078125, -1030.6445645242186),
  new_x_y(-135.51214599609375, -1031.9679987239151),
  new_x_y(-136.68003845214844, -1033.283986323702),
  new_x_y(-137.8842315673828, -1034.590318591403),
  new_x_y(-139.12411499023438, -1035.8849434285803),
  new_x_y(-140.3990478515625, -1037.1659199800672),
  new_x_y(-141.70838928222656, -1038.431439475185),
  new_x_y(-143.05148315429688, -1039.679798522091),
  new_x_y(-144.42764282226562, -1040.9093769333458),
  new_x_y(-145.83311462402344, -1042.1160772946891),
  new_x_y(-147.24366760253906, -1043.2797508701397),
  new_x_y(-148.6542205810547, -1044.398189605361),
  new_x_y(-150.0647735595703, -1045.4734179410927),
  new_x_y(-151.9115447998047, -1046.818879204404),
  new_x_y(-153.56297302246094, -1047.9649649606927),
  new_x_y(-155.27511596679688, -1049.0989259660173),
  new_x_y(-157.0152130126953, -1050.1971999910343),
  new_x_y(-158.78709411621094, -1051.261679878713),
  new_x_y(-160.56326293945312, -1052.2763142840188),
  new_x_y(-162.36941528320312, -1053.256244237683),
  new_x_y(-164.17236328125, -1054.1841280003594),
  new_x_y(-166.06207275390625, -1055.1045175025336),
  new_x_y(-167.92999267578125, -1055.963504145741),
  new_x_y(-169.81085205078125, -1056.7789530599423),
  new_x_y(-171.71499633789062, -1057.555335983251),
  new_x_y(-173.6272430419922, -1058.2865596520683),
  new_x_y(-175.54446411132812, -1058.9721381371337),
  new_x_y(-177.5172576904297, -1059.6290460067014),
  new_x_y(-179.49436950683594, -1060.2390685602497),
  new_x_y(-181.5004119873047, -1060.80958016515),
  new_x_y(-183.51683044433597, -1061.3347975878194),
  new_x_y(-185.5426788330078, -1061.8146020439071),
  new_x_y(-187.5782928466797, -1062.2491491193605),
  new_x_y(-189.625, -1062.6386692708334),
  new_x_y(-191.70413208007807, -1062.9863178358407),
  new_x_y(-193.7519073486328, -1063.2819196173116),
  new_x_y(-195.80239868164065, -1063.5318193259066),
  new_x_y(-197.89547729492188, -1063.7397222603213),
  new_x_y(-199.98516845703125, -1063.9000472976386),
  new_x_y(-202.0535888671875, -1064.0124941221452),
  new_x_y(-204.19088745117188, -1064.0805283660864),
  new_x_y(-206.28028869628903, -1064.0998263672855),
  new_x_y(-208.3740997314453, -1064.0723702297396),
  new_x_y(-210.478515625, -1063.9975325926928),
  new_x_y(-212.55540466308597, -1063.8771264852148),
  new_x_y(-214.64825439453125, -1063.708837278481),
  new_x_y(-216.74729919433597, -1063.4924460365803),
  new_x_y(-218.8514862060547, -1063.2273400866045),
  new_x_y(-220.93853759765625, -1062.9163286544263),
  new_x_y(-223.02557373046875, -1062.5569658086474),
  new_x_y(-225.13626098632807, -1062.1437710248788),
  new_x_y(-227.19061279296875, -1061.6929057110094),
  new_x_y(-229.30372619628903, -1061.1782614004462)
]

SEC_12_WAYPOINTS = [
 new_x_y(-343.2425231933594, 57.59950256347656),
  new_x_y(-343.2458117675781, 59.59837341308594),
  new_x_y(-343.24910034179686, 61.59727478027344),
  new_x_y(-343.2523889160156, 63.59614562988281),
  new_x_y(-343.2556469726562, 65.59504699707031),
  new_x_y(-343.258935546875, 67.59391784667969),
  new_x_y(-343.2622241210937, 69.59281921386719),
  new_x_y(-343.2655126953125, 71.59169006347656),
  new_x_y(-343.26880126953125, 73.59059143066406),
  new_x_y(-343.27208984375, 75.58946228027344),
  new_x_y(-343.27537841796874, 77.58836364746094),
  new_x_y(-343.2786669921875, 79.58723449707031),
  new_x_y(-343.2819555664062, 81.58613586425781),
  new_x_y(-343.2852136230469, 83.58500671386719),
  new_x_y(-343.28850219726564, 85.58390808105469),
  new_x_y(-343.2917907714844, 87.58277893066406),
  new_x_y(-343.29507934570313, 89.58168029785156),
  new_x_y(-343.2983679199219, 91.58058166503906),
  new_x_y(-343.3016564941406, 93.57945251464844),
  new_x_y(-343.30494506835936, 95.57835388183594),
  new_x_y(-343.3082336425781, 97.57722473144533),
  new_x_y(-343.3115222167969, 99.5761260986328),
  new_x_y(-343.3147802734375, 101.5749969482422),
  new_x_y(-343.31806884765626, 103.57389831542967),
  new_x_y(-343.321357421875, 105.57276916503906),
  new_x_y(-343.3246459960937, 107.57167053222656),
  new_x_y(-343.3279345703125, 109.57054138183594),
  new_x_y(-343.33122314453124, 111.56944274902344),
  new_x_y(-343.33451171875, 113.5683135986328),
  new_x_y(-343.3378002929687, 115.56721496582033),
  new_x_y(-343.3410888671875, 117.56608581542967),
  new_x_y(-343.34434692382814, 119.5649871826172),
  new_x_y(-343.3476354980469, 121.56385803222656),
  new_x_y(-343.3509240722656, 123.56275939941406),
  new_x_y(-343.35421264648437, 125.56166076660156),
  new_x_y(-343.3575012207031, 127.56053161621094),
  new_x_y(-343.36078979492186, 129.55943298339844),
  new_x_y(-343.3640783691406, 131.5583038330078),
  new_x_y(-343.3673669433594, 133.5572052001953),
  new_x_y(-343.370625, 135.5560760498047),
  new_x_y(-343.37391357421876, 137.5549774169922),
  new_x_y(-343.3072021484375, 139.55384826660156),
  new_x_y(-343.24049072265626, 141.55274963378906),
  new_x_y(-343.173779296875, 143.55162048339844),
  new_x_y(-343.1070678710937, 145.55052185058594),
  new_x_y(-343.0403564453125, 147.5493927001953),
  new_x_y(-342.97364501953126, 149.5482940673828),
  new_x_y(-342.90693359375, 151.5471649169922),
  new_x_y(-342.84019165039064, 153.5460662841797),
  new_x_y(-342.7734802246094, 155.54493713378906),
  new_x_y(-342.70676879882814, 157.54383850097656),
  new_x_y(-342.6400573730469, 159.54270935058594),
  new_x_y(-342.57334594726564, 161.54161071777344),
  new_x_y(-342.5066345214844, 163.54051208496094),
  new_x_y(-342.43992309570314, 165.5393829345703),
  new_x_y(-342.3732116699219, 167.5382843017578),
  new_x_y(-342.30650024414064, 169.5371551513672),
  new_x_y(-342.23975830078126, 171.5360565185547),
  new_x_y(-342.173046875, 173.53492736816406),
  new_x_y(-342.10633544921876, 175.53382873535156),
  new_x_y(-342.0396240234375, 177.53269958496094),
  new_x_y(-341.97291259765626, 179.53160095214844),
  new_x_y(-341.906201171875, 181.5304718017578),
  new_x_y(-341.8394897460937, 183.5293731689453),
  new_x_y(-341.7727783203125, 185.5282440185547),
  new_x_y(-341.70606689453126, 187.5271453857422),
  new_x_y(-341.6393249511719, 189.5260162353516),
  new_x_y(-341.57261352539064, 191.52491760253903),
  new_x_y(-341.5059020996094, 193.52378845214844),
  new_x_y(-341.43919067382814, 195.52268981933597),
  new_x_y(-341.3724792480469, 197.52159118652344),
  new_x_y(-341.30576782226564, 199.5204620361328),
  new_x_y(-341.2390563964844, 201.5193634033203),
  new_x_y(-341.17234497070314, 203.5182342529297),
  new_x_y(-341.1056335449219, 205.5171356201172),
  new_x_y(-341.0388916015625, 207.51600646972656),
  new_x_y(-340.97218017578126, 209.5149078369141),
  new_x_y(-340.90546875, 211.51377868652344),
  new_x_y(-340.83875732421876, 213.5126800537109),
  new_x_y(-340.7720458984375, 215.5115509033203),
  new_x_y(-340.70533447265626, 217.5104522705078),
  new_x_y(-340.638623046875, 219.5093231201172),
  new_x_y(-340.5719116210937, 221.5082244873047),
  new_x_y(-340.5052001953125, 223.5070953369141),
  new_x_y(-340.43845825195314, 225.5059967041016),
  new_x_y(-340.3717468261719, 227.5048675537109),
  new_x_y(-340.30503540039064, 229.50376892089844),
  new_x_y(-340.2383239746094, 231.50267028808597),
  new_x_y(-340.17161254882814, 233.5015411376953),
  new_x_y(-340.1049011230469, 235.5004425048828),
  new_x_y(-340.03818969726564, 237.4993133544922),
  new_x_y(-339.9714782714844, 239.4982147216797),
  new_x_y(-339.90476684570314, 241.49708557128903),
  new_x_y(-339.8380249023437, 243.49598693847656),
  new_x_y(-339.7713134765625, 245.49485778808597),
  new_x_y(-339.70460205078126, 247.49375915527344),
  new_x_y(-339.637890625, 249.4926300048828),
  new_x_y(-339.57117919921876, 251.4915313720703),
  new_x_y(-339.5044677734375, 253.4904022216797),
  new_x_y(-339.43775634765626, 255.4893035888672),
  new_x_y(-339.371044921875, 257.4881591796875),
  new_x_y(-339.3043334960937, 259.487060546875),
  new_x_y(-339.2375915527344, 261.4859619140625),
  new_x_y(-339.17088012695314, 263.48486328125),
  new_x_y(-339.1041687011719, 265.48370361328125),
  new_x_y(-339.03745727539064, 267.48260498046875),
  new_x_y(-338.9707458496094, 269.48150634765625),
  new_x_y(-338.90403442382814, 271.4804077148437),
  new_x_y(-338.8373229980469, 273.479248046875),
  new_x_y(-338.77061157226564, 275.4781494140625),
  new_x_y(-338.70386962890626, 277.47705078125),
  new_x_y(-338.637158203125, 279.4759521484375),
  new_x_y(-338.5704467773437, 281.47479248046875),
  new_x_y(-338.5037353515625, 283.47369384765625),
  new_x_y(-338.43702392578126, 285.4725952148437),
  new_x_y(-338.3699462890625, 287.4714660644531),
  new_x_y(-338.2862670898437, 289.4696960449219),
  new_x_y(-338.177685546875, 291.4667358398437),
  new_x_y(-337.74420166015625, 293.4622802734375),
  new_x_y(-337.561151551239, 295.57938104829776),
  new_x_y(-337.3687309999671, 297.6956472447914),
  new_x_y(-337.15758115312997, 299.81011987990416),
  new_x_y(-336.91836908806914, 301.9215914330887),
  new_x_y(-336.6418076495959, 304.0284823421357),
  new_x_y(-336.318683504231, 306.12871865486255),
  new_x_y(-335.9398960929732, 308.21961161765273),
  new_x_y(-335.4965100939307, 310.29774031792573),
  new_x_y(-334.9798238937089, 312.3588389125448),
  new_x_y(-334.38145639508144, 314.39769046277337),
  new_x_y(-333.69345423906987, 316.40802995065553),
  new_x_y(-332.90842117070656, 318.38245965990507),
  new_x_y(-332.01967080638593, 320.3123807501063),
  new_x_y(-331.02140344302677, 322.18794551404585),
  new_x_y(-329.9089067619942, 323.99803545528476),
  new_x_y(-328.67877930271305, 325.7302709198313),
  new_x_y(-327.3291743951622, 327.37105851663097),
  new_x_y(-325.8600608366352, 328.90568291218955),
  new_x_y(-324.27349497551563, 330.3184497216768),
  new_x_y(-322.57389703549154, 331.5928860707134),
  new_x_y(-320.7683225063256, 332.7120048904),
  new_x_y(-318.86671729112027, 333.6586380505436),
  new_x_y(-320.55279541015625, 332.7275085449219),
  new_x_y(-318.68414306640625, 333.4375305175781),
  new_x_y(-316.74951171875, 333.9408569335937),
  new_x_y(-314.7717590332031, 334.2315979003906),
  new_x_y(-312.7741394042969, 334.30633544921875),
  new_x_y(-310.7801818847656, 334.16412353515625),
  new_x_y(-308.80279541015625, 333.8643798828125),
  new_x_y(-307.814697265625, 333.7107238769531),
  new_x_y(-306.8265686035156, 333.5570678710937),
  new_x_y(-305.83843994140625, 333.4034423828125),
  new_x_y(-304.8503112792969, 333.2497863769531),
  new_x_y(-303.8621826171875, 333.0961303710937),
  new_x_y(-301.877197265625, 332.8551025390625),
  new_x_y(-299.88006591796875, 332.75640869140625),
  new_x_y(-297.8809814453125, 332.8007202148437),
  new_x_y(-295.8901672363281, 332.9877319335937),
  new_x_y(-293.9178161621094, 333.3164978027344),
  new_x_y(-291.9739990234375, 333.7853698730469),
  new_x_y(-290.0686340332031, 334.3919372558594),
  new_x_y(-287.36890955136124, 336.4502300627805),
  new_x_y(-285.53462028523313, 337.43563207506764),
  new_x_y(-283.8206015721414, 338.6180705351394),
  new_x_y(-282.2413695974792, 339.97542718319943),
  new_x_y(-280.8073997467029, 341.48555109159435),
  new_x_y(-279.52547159263526, 343.12681286859697),
  new_x_y(-278.3990548588583, 344.87855052651463),
  new_x_y(-277.4287151511894, 346.7214132459724),
  new_x_y(-276.6125219176662, 348.63761172938075),
  new_x_y(-275.9464446647921, 350.61108536348434),
  new_x_y(-275.424726785363, 352.62759715614163),
  new_x_y(-275.04022934994344, 354.6747675289015),
  new_x_y(-274.78473982892007, 356.7420576792383),
  new_x_y(-274.6492429260599, 358.8207125056033),
  new_x_y(-274.62415252333415, 360.9036721291275),
  new_x_y(-274.6995051839878, 362.98545994418),
  new_x_y(-274.86511677200605, 365.06205396436957),
  new_x_y(-275.11070456392184, 367.1307470623927),
  new_x_y(-275.42597779922124, 369.1900005776115),
  new_x_y(-275.8006999845239, 371.23929471755116),
  new_x_y(-276.22472647836435, 373.2789782309583),
  new_x_y(-276.68802097838517, 375.31011899439363),
  new_x_y(-277.18065454720767, 377.334356438785),
  new_x_y(-277.6927907782895, 379.3537561495886),
  new_x_y(-278.21466064453125, 381.3706665039063),
  new_x_y(-278.75653076171875, 383.2958679199219),
  new_x_y(-279.324462890625, 385.2135009765625),
  new_x_y(-279.9183959960937, 387.1232604980469),
  new_x_y(-280.5381774902344, 389.0247802734375),
  new_x_y(-281.29998779296875, 391.6999816894531),
  new_x_y(-282.1494140625, 393.758056640625)
]

#new points for testing purposes 

SEC_1_WAYPOINTS = [   
 new_x_y(-283.79998779296875, 391.6999816894531),
 new_x_y(-284.6494140625, 393.758056640625),
 new_x_y(-289.4931335449219, 407.9544677734375),
 new_x_y(-290.13897705078125, 409.8473205566406),
 new_x_y(-290.7847900390625, 411.74017333984375),
 new_x_y(-291.4306335449219, 413.633056640625),
 new_x_y(-292.0764465332031, 415.5259094238281),
 new_x_y(-292.7222900390625, 417.41876220703125),
 new_x_y(-293.36810302734375, 419.3116149902344),
 new_x_y(-294.0139465332031, 421.2044677734375),
 new_x_y(-294.6597595214844, 423.0973205566406),
 new_x_y(-295.30560302734375, 424.99017333984375),
 new_x_y(-295.951416015625, 426.883056640625),
 new_x_y(-296.5972595214844, 428.7759094238281),
 new_x_y(-297.2430725097656, 430.66876220703125),
 new_x_y(-297.888916015625, 432.5616149902344),
 new_x_y(-298.53472900390625, 434.4544677734375),
 new_x_y(-299.1805725097656, 436.3473205566406),
 new_x_y(-299.8263854980469, 438.24017333984375),
 new_x_y(-300.41998291015625, 439.97998046875),
 new_x_y(-301.0658264160156, 441.8728332519531),
 new_x_y(-301.7116394042969, 443.76568603515625),
 new_x_y(-302.35748291015625, 445.6585388183594),
 new_x_y(-303.0032958984375, 447.5513916015625),
 new_x_y(-303.6491394042969, 449.44427490234375),
 new_x_y(-304.2949523925781, 451.3371276855469),
 new_x_y(-304.9407958984375, 453.22998046875),
 new_x_y(-305.58660888671875, 455.1228332519531),
 new_x_y(-306.2324523925781, 457.01568603515625),
 new_x_y(-306.8782653808594, 458.9085388183594),
 new_x_y(-307.52410888671875, 460.8013916015625),
 new_x_y(-308.169921875, 462.6942443847656),
 new_x_y(-308.8157653808594, 464.5871276855469),
 new_x_y(-309.4615783691406, 466.47998046875),
 new_x_y(-310.10736083984375, 468.37286376953125),
 new_x_y(-310.746337890625, 470.26800537109375),
 new_x_y(-311.374267578125, 472.1669006347656),
 new_x_y(-311.9910583496094, 474.06939697265625),
 new_x_y(-312.5967712402344, 475.9754638671875),
 new_x_y(-313.1913146972656, 477.88507080078125),
 new_x_y(-313.7746887207031, 479.798095703125),
 new_x_y(-314.3468933105469, 481.7144775390625),
 new_x_y(-314.90789794921875, 483.6341857910156),
 new_x_y(-315.4576721191406, 485.55712890625),
 new_x_y(-315.9962158203125, 487.4832458496094),
 new_x_y(-316.52349853515625, 489.4124755859375),
 new_x_y(-317.03948974609375, 491.34478759765625),
 new_x_y(-317.544189453125, 493.280029296875),
 new_x_y(-318.03759765625, 495.2182312011719),
 new_x_y(-318.5196533203125, 497.1592712402344),
 new_x_y(-318.9903869628906, 499.10308837890625),
 new_x_y(-319.4497375488281, 501.0495910644531),
 new_x_y(-319.8977355957031, 502.998779296875),
 new_x_y(-320.3343200683594, 504.9505615234375),
 new_x_y(-320.759521484375, 506.9048156738281),
 new_x_y(-321.17327880859375, 508.8615417480469),
 new_x_y(-321.57562255859375, 510.8206787109375),
 new_x_y(-321.96649169921875, 512.7821044921875),
 new_x_y(-322.3459167480469, 514.7457885742188),
 new_x_y(-322.7138671875, 516.7116088867188),
 new_x_y(-323.0703125, 518.6796264648438),
 new_x_y(-323.4152526855469, 520.649658203125),
 new_x_y(-323.74871826171875, 522.6216430664062),
 new_x_y(-324.07061767578125, 524.5955810546875),
 new_x_y(-324.3810119628906, 526.5712890625),
 new_x_y(-324.678466796875, 528.549072265625),
 new_x_y(-324.96136474609375, 530.5289916992188),
 new_x_y(-325.22967529296875, 532.5109252929688),
 new_x_y(-325.4833984375, 534.4947509765625),
 new_x_y(-325.7225036621094, 536.4804077148438),
 new_x_y(-325.947021484375, 538.4677124023438),
 new_x_y(-326.1568908691406, 540.4566650390625),
 new_x_y(-326.3521423339844, 542.4471435546875),
 new_x_y(-326.53271484375, 544.43896484375),
 new_x_y(-326.6986389160156, 546.4320678710938),
 new_x_y(-326.84991455078125, 548.4263305664062),
 new_x_y(-326.9864807128906, 550.421630859375),
 new_x_y(-327.1083679199219, 552.4179077148438),
 new_x_y(-327.215576171875, 554.4150390625),
 new_x_y(-327.30810546875, 556.4129028320312),
 new_x_y(-327.3858947753906, 558.411376953125),
 new_x_y(-327.4490051269531, 560.410400390625),
 new_x_y(-327.49737548828125, 562.4097900390625),
 new_x_y(-327.53106689453125, 564.4094848632812),
 new_x_y(-327.550048828125, 566.409423828125),
 new_x_y(-327.5542907714844, 568.409423828125),
 new_x_y(-327.5438232421875, 570.4093627929688),
 new_x_y(-327.5186462402344, 572.4092407226562),
 new_x_y(-327.478759765625, 574.4088134765625),
 new_x_y(-327.42413330078125, 576.4080810546875),
 new_x_y(-327.3548278808594, 578.4068603515625),
 new_x_y(-327.27081298828125, 580.4050903320312),
 new_x_y(-327.1720886230469, 582.4026489257812),
 new_x_y(-327.0586853027344, 584.3994140625),
 new_x_y(-326.93060302734375, 586.3953247070312),
 new_x_y(-326.7931823730469, 588.3905639648438),
 new_x_y(-326.6556701660156, 590.3858032226562),
 new_x_y(-326.51812744140625, 592.381103515625),
 new_x_y(-326.3805847167969, 594.3763427734375),
 new_x_y(-326.2430725097656, 596.37158203125),
 new_x_y(-326.10552978515625, 598.3668823242188),
 new_x_y(-325.9679870605469, 600.3621215820312),
 new_x_y(-325.8304748535156, 602.357421875),
 new_x_y(-325.6914367675781, 604.3526000976562),
 new_x_y(-325.54193115234375, 606.3469848632812),
 new_x_y(-325.3805236816406, 608.3404541015625),
 new_x_y(-325.2071838378906, 610.3329467773438),
 new_x_y(-325.02197265625, 612.3243408203125),
 new_x_y(-324.8248291015625, 614.3145751953125),
 new_x_y(-324.6158447265625, 616.3036499023438),
 new_x_y(-324.3949279785156, 618.2913818359375),
 new_x_y(-324.16217041015625, 620.27783203125),
 new_x_y(-323.91754150390625, 622.2628173828125),
 new_x_y(-323.6610412597656, 624.2462768554688),
 new_x_y(-323.3927001953125, 626.2282104492188),
 new_x_y(-323.112548828125, 628.2084350585938),
 new_x_y(-322.8205261230469, 630.18701171875),
 new_x_y(-322.5167236328125, 632.163818359375),
 new_x_y(-322.2010803222656, 634.1387329101562),
 new_x_y(-321.8736572265625, 636.1117553710938),
 new_x_y(-321.5344543457031, 638.082763671875),
 new_x_y(-321.1834716796875, 640.0517578125),
 new_x_y(-320.82073974609375, 642.0185546875),
 new_x_y(-320.4462585449219, 643.9832153320312),
 new_x_y(-320.06005859375, 645.945556640625),
 new_x_y(-319.662109375, 647.9055786132812),
 new_x_y(-319.2524719238281, 649.8631591796875),
 new_x_y(-318.8311462402344, 651.8182373046875),
 new_x_y(-318.3981628417969, 653.7708129882812),
 new_x_y(-317.9534912109375, 655.7207641601562),
 new_x_y(-317.4971923828125, 657.6680297851562),
 new_x_y(-317.029052734375, 659.6124877929688),
 new_x_y(-316.5418395996094, 661.5521850585938),
 new_x_y(-316.03179931640625, 663.486083984375),
 new_x_y(-315.4990234375, 665.4137573242188),
 new_x_y(-314.943603515625, 667.3350830078125),
 new_x_y(-314.36553955078125, 669.2496948242188),
 new_x_y(-313.7650146484375, 671.1574096679688),
 new_x_y(-313.14202880859375, 673.0579223632812),
 new_x_y(-312.4966735839844, 674.950927734375),
 new_x_y(-311.8291015625, 676.836181640625),
 new_x_y(-311.13934326171875, 678.7135009765625),
 new_x_y(-310.42755126953125, 680.58251953125),
 new_x_y(-309.69378662109375, 682.4430541992188),
 new_x_y(-308.9381103515625, 684.2947998046875),
 new_x_y(-308.16070556640625, 686.1375122070312),
 new_x_y(-307.3616638183594, 687.970947265625),
 new_x_y(-306.54107666015625, 689.7947998046875),
 new_x_y(-305.69903564453125, 691.60888671875),
 new_x_y(-304.835693359375, 693.4129638671875),
 new_x_y(-303.951171875, 695.2067260742188),
 new_x_y(-303.04559326171875, 696.9899291992188),
 new_x_y(-302.1190490722656, 698.7623291015625),
 new_x_y(-301.17169189453125, 700.5237426757812),
 new_x_y(-300.20367431640625, 702.2738647460938),
 new_x_y(-299.215087890625, 704.012451171875),
 new_x_y(-298.2060852050781, 705.7392578125),
 new_x_y(-297.17681884765625, 707.4540405273438),
 new_x_y(-296.12744140625, 709.1566162109375),
 new_x_y(-295.0580749511719, 710.8466796875),
 new_x_y(-293.9797668457031, 712.5310668945312),
 new_x_y(-292.9014587402344, 714.2155151367188),
 new_x_y(-291.8224182128906, 715.8994140625),
 new_x_y(-290.7375793457031, 717.5797119140625),
 new_x_y(-289.6460266113281, 719.2554931640625),
 new_x_y(-288.54779052734375, 720.927001953125),
 new_x_y(-287.44287109375, 722.5941162109375),
 new_x_y(-286.331298828125, 724.2567138671875),
 new_x_y(-285.21307373046875, 725.9149169921875),
 new_x_y(-284.0882568359375, 727.568603515625),
 new_x_y(-282.956787109375, 729.2178344726562),
 new_x_y(-281.8187561035156, 730.8624877929688),
 new_x_y(-280.6741638183594, 732.5025634765625),
 new_x_y(-279.52301025390625, 734.1380615234375),
 new_x_y(-278.3653259277344, 735.7689208984375),
 new_x_y(-277.20111083984375, 737.3951416015625),
 new_x_y(-276.0304260253906, 739.0167236328125),
 new_x_y(-274.8532409667969, 740.633544921875),
 new_x_y(-273.66961669921875, 742.2457275390625),
 new_x_y(-272.47955322265625, 743.8531494140625),
 new_x_y(-271.2830505371094, 745.4557495117188),
 new_x_y(-270.0801696777344, 747.0535888671875),
 new_x_y(-268.8708801269531, 748.6466064453125),
 new_x_y(-267.6552429199219, 750.2347412109375),
 new_x_y(-266.4332580566406, 751.8179931640625),
 new_x_y(-265.2049560546875, 753.3963623046875),
 new_x_y(-263.9703674316406, 754.9698486328125),
 new_x_y(-262.7294616699219, 756.538330078125),
 new_x_y(-261.48297119140625, 758.1024169921875),
 new_x_y(-260.2355041503906, 759.6657104492188),
 new_x_y(-258.9880676269531, 761.2289428710938),
 new_x_y(-257.7406005859375, 762.792236328125),
 new_x_y(-256.4931640625, 764.3555297851562),
 new_x_y(-255.24571228027344, 765.9188232421875),
 new_x_y(-253.99826049804688, 767.4821166992188),
 new_x_y(-252.7508087158203, 769.04541015625),
 new_x_y(-251.49908447265625, 770.605224609375),
 new_x_y(-250.23558044433594, 772.1555786132812),
 new_x_y(-248.960205078125, 773.6961669921875)
]


# Section 0: 319
# Section 1: 175
# Section 2: 114
# Section 3: 146
# Section 4: 124
# Section 5: 77
# Section 6: 313
# Section 7: 211
# Section 8: 254
# Section 9: 75
# Section 10: 238
# Section 11: 189
# Section 12: 161
# Section 0: 207
# Section 1: 162
# Section 2: 112
# Section 3: 146
# Section 4: 124
# Section 5: 75
# Section 6: 315
# Section 7: 213
# Section 8: 252
# Section 9: 75
# Section 10: 241
# Section 11: 188
# Section 12: 163
# Section 0: 207
# Section 1: 162
# Section 2: 112
# Section 3: 146
# Section 4: 124
# Section 5: 77
# Section 6: 314
# Section 7: 212
# Section 8: 252
# Section 9: 75
# Section 10: 239
# Section 11: 188
# Section 12: 164
# end of the loop
# done
# Solution finished in 347.2500000000506 seconds
#testing  