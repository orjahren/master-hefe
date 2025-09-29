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

Enhanced scenario:
- Increased background traffic with varying speeds to create a more crowded environment.
- Challenging weather conditions (heavy rain, fog, strong winds) to reduce visibility and grip.
- Nighttime setting to further decrease visibility.
- Randomization of speeds and trigger distances for increased unpredictability.
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

        self._velocity = 40  # Base velocity for the cutting-in vehicle
        self._delta_velocity = 10
        self._trigger_distance = 30

        # Enhanced scenario parameters
        self._num_background_vehicles = 8  # Number of additional background vehicles
        self._background_vehicles = []     # List to hold background vehicles
        self._ego_base_velocity = 50       # Base velocity for ego, for calculating relative speeds

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
            # More extreme randomization for primary cut-in vehicle
            self._velocity = random.randint(25, 70)  # Wider speed range
            self._trigger_distance = random.randint(15, 50) # Wider trigger distance range
            self._delta_velocity = random.randint(5, 15) # Randomize acceleration difference

    def _initialize_actors(self, config):

        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'
        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add the primary cutting-in actor from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            if vehicle is not None:
                self.other_actors.append(vehicle)
                vehicle.set_simulate_physics(enabled=False) # Temporarily disable for precise initial placement
            else:
                print(f"Warning: Could not spawn primary cut-in vehicle at {actor.transform.location}")


        # transform visible for primary cutting-in actor
        if self.other_actors:
            other_actor_transform = self.other_actors[0].get_transform()
            self._transform_visible = carla.Transform(
                carla.Location(other_actor_transform.location.x,
                               other_actor_transform.location.y,
                               other_actor_transform.location.z + 105),
                other_actor_transform.rotation)
        else:
            print("Error: No primary cut-in actor spawned.")
            # Fallback to a default visible transform if no actor was spawned
            self._transform_visible = carla.Transform(
                carla.Location(config.trigger_points[0].location.x,
                               config.trigger_points[0].location.y,
                               config.trigger_points[0].location.z + 105),
                config.trigger_points[0].rotation)


        # --- Add Background Traffic for increased complexity ---
        ego_transform = self.ego_vehicles[0].get_transform()
        ego_waypoint = self._map.get_waypoint(ego_transform.location, project_to_road=True)

        spawn_points = self._map.get_spawn_points()
        random.shuffle(spawn_points)

        valid_spawn_points = []
        for sp in spawn_points:
            wp = self._map.get_waypoint(sp.location, project_to_road=True)
            # Ensure it's on a driving lane, not a junction, and within a reasonable distance
            if wp and wp.is_junction is False and wp.lane_type == carla.LaneType.Driving:
                # Distance checks to ensure vehicles are somewhat around the area of interest
                dist_from_ego = sp.location.distance(ego_transform.location)
                if self.other_actors:
                    dist_from_cut_in = sp.location.distance(self.other_actors[0].get_transform().location)
                else:
                    dist_from_cut_in = float('inf') # If no cut-in car, treat as very far

                # Spawn points between 50 to 300 meters from ego AND the cut-in car
                if (50 < dist_from_ego < 300) and (50 < dist_from_cut_in < 300):
                    valid_spawn_points.append(sp)

        # Limit to available valid spawn points
        self._num_background_vehicles = min(self._num_background_vehicles, len(valid_spawn_points))

        for i in range(self._num_background_vehicles):
            spawn_point = valid_spawn_points[i]
            # Request a random vehicle model
            vehicle = CarlaDataProvider.request_new_actor('vehicle.*', spawn_point)
            if vehicle is not None:
                self.other_actors.append(vehicle)
                self._background_vehicles.append(vehicle)
                # Physics are enabled by default, no need to set explicitly unless disabling.
            else:
                print(f"Warning: Could not spawn background vehicle at {spawn_point.location}")

        # --- Set challenging weather and time of day ---
        self.world.set_weather(carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=90.0,
            precipitation_deposits=50.0,
            wind_intensity=70.0,
            sun_altitude_angle=-20.0, # Nighttime
            fog_density=40.0,
            fog_distance=70.0,
            wetness=60.0,
            scattering_intensity=1.0, # For better fog effect
            mie_scattering_scale=0.0
        ))
        # Ensure ego's initial speed is set
        if self.ego_vehicles:
            self.ego_vehicles[0].set_velocity(carla.Vector3D(
                self._ego_base_velocity * ego_transform.get_forward_vector().x,
                self._ego_base_velocity * ego_transform.get_forward_vector().y,
                0))


    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance

        Enhanced behavior:
        - All previous steps for the cutting-in car run in parallel with additional background traffic
          driving with varying speeds and challenging environmental conditions.
        """

        # Overall root behavior, running all components in parallel
        # The scenario completes when all parallel branches succeed
        overall_scenario_root = py_trees.composites.Parallel(
            "Overall Scenario Complexity", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        # --- Main Cut-in behavior sequence for the primary actor ---
        cut_in_sequence = py_trees.composites.Sequence("CutIn Main Behavior")

        if self.other_actors:
            # car_visible: spawn car at a visible transform
            car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
            cut_in_sequence.add_child(car_visible)

            # just_drive: drive until in trigger distance to ego_vehicle
            just_drive = py_trees.composites.Parallel(
                "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            car_driving = WaypointFollower(self.other_actors[0], self._velocity,
                                           name="CutInCar_DriveUntilTrigger")
            just_drive.add_child(car_driving)
            trigger_distance = InTriggerDistanceToVehicle(
                self.other_actors[0], self.ego_vehicles[0], self._trigger_distance,
                name="CutInCar_TriggerDistance")
            just_drive.add_child(trigger_distance)
            cut_in_sequence.add_child(just_drive)

            # accelerate: accelerate to catch up distance to ego_vehicle
            accelerate = AccelerateToCatchUp(self.other_actors[0], self.ego_vehicles[0], throttle_value=1.0,
                                             delta_velocity=self._delta_velocity, trigger_distance=5, max_distance=500,
                                             name="CutInCar_Accelerate")
            cut_in_sequence.add_child(accelerate)

            # lane_change: change the lane
            if self._direction == 'left':
                lane_change = LaneChange(
                    self.other_actors[0], speed=None, direction='right', distance_same_lane=5, distance_other_lane=300,
                    name="CutInCar_LaneChangeRight")
            else: # self._direction == 'right'
                lane_change = LaneChange(
                    self.other_actors[0], speed=None, direction='left', distance_same_lane=5, distance_other_lane=300,
                    name="CutInCar_LaneChangeLeft")
            cut_in_sequence.add_child(lane_change)

            # endcondition: drive for a defined distance after lane change
            endcondition = DriveDistance(self.other_actors[0], 200, name="CutInCar_EndDrive")
            cut_in_sequence.add_child(endcondition)
        else:
            print("Warning: No primary cut-in actor, main cut-in sequence will not run.")
            # If no cut-in actor, ensure this branch still "succeeds" to not block the root
            cut_in_sequence.add_child(py_trees.behaviours.Success("NoCutInActor"))

        overall_scenario_root.add_child(cut_in_sequence)

        # --- Background traffic behavior ---
        if self._background_vehicles:
            background_traffic_parallel = py_trees.composites.Parallel(
                "Background Traffic Driving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
            for i, vehicle in enumerate(self._background_vehicles):
                # Vary speeds for background traffic for more dynamic behavior
                # Speeds relative to ego's base velocity for more natural traffic flow
                background_speed = random.uniform(self._ego_base_velocity * 0.7, self._ego_base_velocity * 1.3)
                background_traffic_parallel.add_child(
                    WaypointFollower(vehicle, background_speed, name=f"BackgroundCar_{i}_Follow"))
            overall_scenario_root.add_child(background_traffic_parallel)

        return overall_scenario_root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        # Collision with any other actor (ego vehicle is self.ego_vehicles[0])
        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors after deletion.
        """
        self.remove_all_actors()
```