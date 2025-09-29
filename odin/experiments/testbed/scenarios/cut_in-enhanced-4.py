```python
#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Cut in scenario:

The scenario realizes a driving behavior on the highway.
The user-controlled ego vehicle is driving straight and keeping its velocity at a constant level.
Another car is cutting just in front, coming from left or right lane.

The ego vehicle may need to brake to avoid a collision.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      LaneChange,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario


class CutIn(BasicScenario):

    """
    The ego vehicle is driving on a highway and another car is cutting in just in front.
    This is a single ego vehicle scenario
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        # Enhanced parameters for increased complexity
        self._velocity = 45  # Slightly higher ego velocity for faster highway driving
        self._delta_velocity = 15  # Cut-in vehicle accelerates more aggressively
        self._trigger_distance = 25  # Cut-in vehicle initiates lane change closer to ego

        # New: Parameters for additional background traffic to increase scene density
        self._lead_vehicle_offset = 35   # meters ahead of ego in the same lane
        self._tail_vehicle_offset = -25  # meters behind ego in the same lane
        self._lead_vehicle_speed_diff = -10 # km/h slower than ego's intended speed
        self._tail_vehicle_speed_diff = 5  # km/h faster than ego's intended speed

        self._lead_vehicle = None
        self._tail_vehicle = None

        # New: Adverse weather conditions to decrease visibility and grip
        self._weather_params = carla.WeatherParameters(
            cloudiness=70.0,
            precipitation=30.0,
            precipitation_deposits=25.0,
            wind_intensity=12.0,
            fog_density=8.0,
            wetness=55.0,
            sun_azimuth_angle=270.0,  # Evening, low sun angle
            sun_altitude_angle=15.0   # Potentially causing glare
        )
        CarlaDataProvider.get_world().set_weather(self._weather_params)

        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(CutIn, self).__init__("CutIn",
                                    ego_vehicles,
                                    config,
                                    world,
                                    debug_mode,
                                    criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(30, 70)
            self._trigger_distance = random.randint(15, 35)
            self._delta_velocity = random.randint(10, 20)
            self._lead_vehicle_offset = random.randint(30, 50)
            self._tail_vehicle_offset = random.randint(-40, -20)
            self._lead_vehicle_speed_diff = random.randint(-15, -5)
            self._tail_vehicle_speed_diff = random.randint(0, 10)

            # Randomize weather parameters within a challenging range
            self._weather_params.cloudiness = random.uniform(50.0, 90.0)
            self._weather_params.precipitation = random.uniform(10.0, 50.0)
            self._weather_params.fog_density = random.uniform(0.0, 15.0)
            self._weather_params.wetness = random.uniform(30.0, 70.0)
            CarlaDataProvider.get_world().set_weather(self._weather_params)


    def _initialize_actors(self, config):

        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add the primary cut-in actor from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False) # Disable physics initially for precise placement

        # transform visible for the primary cut-in actor
        other_actor_transform = self.other_actors[0].get_transform()
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_transform.location.x,
                           other_actor_transform.location.y,
                           other_actor_transform.location.z + 105), # Spawn high above, then drop
            other_actor_transform.rotation)

        # --- NEW: Spawn additional background vehicles to increase scene complexity ---
        ego_start_location = self.ego_vehicles[0].get_location()
        ego_start_waypoint = self._map.get_waypoint(ego_start_location, lane_type=carla.LaneType.Driving)

        # Lead Vehicle in ego's lane: positioned ahead of ego, driving slower
        lead_waypoint = ego_start_waypoint
        for _ in range(int(self._lead_vehicle_offset / 5)): # Iterate to find a waypoint further ahead
            next_waypoints = lead_waypoint.next(5.0)
            if next_waypoints:
                lead_waypoint = next_waypoints[0]
            else:
                break # End of road, cannot place

        if lead_waypoint:
            # Ensure vehicle spawns slightly above ground to prevent clipping
            lead_transform = carla.Transform(lead_waypoint.transform.location + carla.Location(z=0.5), lead_waypoint.transform.rotation)
            self._lead_vehicle = CarlaDataProvider.request_new_actor('vehicle.audi.a2', lead_transform)
            if self._lead_vehicle:
                self._lead_vehicle.set_simulate_physics(enabled=False)
                self.other_actors.append(self._lead_vehicle) # Add to general actor list for cleanup

        # Tail Vehicle in ego's lane: positioned behind ego, driving faster
        tail_waypoint = ego_start_waypoint
        for _ in range(int(abs(self._tail_vehicle_offset) / 5)): # Iterate to find a waypoint further behind
            previous_waypoints = tail_waypoint.previous(5.0)
            if previous_waypoints:
                tail_waypoint = previous_waypoints[0]
            else:
                break # Start of road, cannot place

        if tail_waypoint:
            # Ensure vehicle spawns slightly above ground
            tail_transform = carla.Transform(tail_waypoint.transform.location + carla.Location(z=0.5), tail_waypoint.transform.rotation)
            self._tail_vehicle = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', tail_transform)
            if self._tail_vehicle:
                self._tail_vehicle.set_simulate_physics(enabled=False)
                self.other_actors.append(self._tail_vehicle)
        # --- END NEW ---


    def _create_behavior(self):
        """
        Order of sequence for the primary cut-in vehicle:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """

        # Behavior tree for the primary cut-in vehicle (more aggressive)
        cut_in_car_behavior = py_trees.composites.Sequence("CutInCarBehavior")

        # Initial placement of the cut-in vehicle
        car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
        cut_in_car_behavior.add_child(car_visible)

        # Drive straight until within trigger distance to ego_vehicle
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)
        # Trigger distance is now closer, increasing urgency
        trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance)
        just_drive.add_child(trigger_distance)
        cut_in_car_behavior.add_child(just_drive)

        # Accelerate to catch up with ego_vehicle (now more aggressive acceleration)
        accelerate = AccelerateToCatchUp(self.other_actors[0], self.ego_vehicles[0], throttle_value=1,
                                         delta_velocity=self._delta_velocity, trigger_distance=5, max_distance=500)
        cut_in_car_behavior.add_child(accelerate)

        # Perform the lane change
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], speed=None, direction='right', distance_same_lane=5, distance_other_lane=300)
            cut_in_car_behavior.add_child(lane_change)
        else: # _direction == 'right'
            lane_change = LaneChange(
                self.other_actors[0], speed=None, direction='left', distance_same_lane=5, distance_other_lane=300)
            cut_in_car_behavior.add_child(lane_change)

        # After lane change, drive for a defined distance to complete the maneuver
        endcondition = DriveDistance(self.other_actors[0], 200)
        cut_in_car_behavior.add_child(endcondition)


        # --- NEW: Behavior for additional background traffic ---
        background_traffic_behavior = py_trees.composites.Parallel(
            "BackgroundTrafficBehavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        # Lead vehicle behavior: drives slower than ego
        if self._lead_vehicle:
            lead_vehicle_driving = WaypointFollower(self._lead_vehicle, self._velocity + self._lead_vehicle_speed_diff)
            background_traffic_behavior.add_child(lead_vehicle_driving)

        # Tail vehicle behavior: drives faster than ego, closing in
        if self._tail_vehicle:
            tail_vehicle_driving = WaypointFollower(self._tail_vehicle, self._velocity + self._tail_vehicle_speed_diff)
            background_traffic_behavior.add_child(tail_vehicle_driving)
        # --- END NEW ---


        # Combine all behaviors into a single root tree
        # The scenario completes when the primary cut-in behavior finishes (SUCCESS_ON_ONE)
        root = py_trees.composites.Parallel("OverallScenario", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(cut_in_car_behavior)

        # Add background traffic only if vehicles were successfully spawned
        if background_traffic_behavior.children:
            root.add_child(background_traffic_behavior)

        return root


    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors after deletion.
        """
        self.remove_all_actors()
```