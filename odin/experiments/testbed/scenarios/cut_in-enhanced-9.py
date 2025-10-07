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

        self._velocity = 40
        self._delta_velocity = 10
        self._trigger_distance = 30

        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None
        self._third_actor_visible_transform = None  # Added for the new actor

        super(CutIn, self).__init__("CutIn",
                                    ego_vehicles,
                                    config,
                                    world,
                                    debug_mode,
                                    criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _initialize_actors(self, config):

        # Set adverse weather conditions to decrease driveability
        self.world.set_weather(carla.WeatherParameters.HardRain) # Enhanced: Added weather

        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add actors from xml file (this is other_actors[0], the cutting-in car)
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        # transform visible for other_actors[0]
        other_actor_transform = self.other_actors[0].get_transform()
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_transform.location.x,
                           other_actor_transform.location.y,
                           other_actor_transform.location.z + 105),
            other_actor_transform.rotation)

        # Add a third dynamic actor to increase traffic complexity (this will be other_actors[1])
        ego_start_waypoint = self._map.get_waypoint(self.ego_vehicles[0].get_location())
        third_actor_lane_waypoint = None

        if self._direction == 'left': # Cutting in from left, new actor on right lane
            third_actor_lane_waypoint = ego_start_waypoint.get_right_lane()
        else: # Cutting in from right, new actor on left lane
            third_actor_lane_waypoint = ego_start_waypoint.get_left_lane()

        if third_actor_lane_waypoint: # Ensure such a lane exists
            # Place the third actor slightly ahead of the ego vehicle
            # Use ego's velocity to estimate a safe initial distance
            advance_distance = self._trigger_distance + (self._velocity * 1.5 / 3.6) # Convert velocity to m/s, add buffer
            third_actor_waypoint = third_actor_lane_waypoint.next(advance_distance)
            
            if third_actor_waypoint:
                third_actor_waypoint = third_actor_waypoint[0] # next() returns a list of waypoints
                third_actor_model = random.choice(CarlaDataProvider.get_catalog_light())
                
                # Initial off-screen transform, similar to how other_actor[0] might be managed before visibility
                initial_off_screen_transform = carla.Transform(
                    carla.Location(third_actor_waypoint.transform.location.x,
                                   third_actor_waypoint.transform.location.y,
                                   third_actor_waypoint.transform.location.z - 500), # Spawn far below ground
                    third_actor_waypoint.transform.rotation)

                third_vehicle = CarlaDataProvider.request_new_actor(third_actor_model, initial_off_screen_transform)
                self.other_actors.append(third_vehicle)
                third_vehicle.set_simulate_physics(enabled=False) # Physics will be enabled by WaypointFollower behavior

                # Store visible transform for ActorTransformSetter in _create_behavior
                self._third_actor_visible_transform = third_actor_waypoint.transform

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        # Behavior for other_actors[0] (the cutting-in car)
        behavior_other_0 = py_trees.composites.Sequence("CutInActorBehavior")
        car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
        behavior_other_0.add_child(car_visible)

        # just_drive for other_actors[0]
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)
        trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance)
        just_drive.add_child(trigger_distance)
        behavior_other_0.add_child(just_drive)

        # accelerate for other_actors[0]
        accelerate = AccelerateToCatchUp(self.other_actors[0], self.ego_vehicles[0], throttle_value=1,
                                         delta_velocity=self._delta_velocity, trigger_distance=5, max_distance=500)
        behavior_other_0.add_child(accelerate)

        # lane_change for other_actors[0]
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], speed=None, direction='right', distance_same_lane=5, distance_other_lane=300)
            behavior_other_0.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self.other_actors[0], speed=None, direction='left', distance_same_lane=5, distance_other_lane=300)
            behavior_other_0.add_child(lane_change)

        # Create a parallel composite to run all actor behaviors simultaneously
        all_actors_behaviors = py_trees.composites.Parallel(
            "AllActorsDriving", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        all_actors_behaviors.add_child(behavior_other_0)

        # Add behavior for other_actors[1] (the additional traffic car) if it was spawned
        if len(self.other_actors) > 1: # Enhanced: Added another dynamic actor
            behavior_other_1 = py_trees.composites.Sequence("ThirdActorBehavior")
            third_actor_visible_setter = ActorTransformSetter(self.other_actors[1], self._third_actor_visible_transform)
            behavior_other_1.add_child(third_actor_visible_setter)
            # Make it drive slightly slower than ego/cutting-in car for more interaction
            third_actor_driving = WaypointFollower(self.other_actors[1], self._velocity - 5)
            behavior_other_1.add_child(third_actor_driving)
            # Ensure it drives for a sufficient duration, longer than the scenario's main end condition
            behavior_other_1.add_child(DriveDistance(self.other_actors[1], 500)) 
            
            all_actors_behaviors.add_child(behavior_other_1)

        # The end condition for the overall scenario (based on the cutting-in car)
        endcondition = DriveDistance(self.other_actors[0], 200)

        # Build the root tree: all actor behaviors run in parallel, and the scenario ends
        # when either all parallel behaviors complete or the primary endcondition is met.
        root = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(all_actors_behaviors)
        root.add_child(endcondition)
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