```python
#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Complex Cut-in scenario:

The scenario realizes a driving behavior on the highway with increased complexity.
The user-controlled ego vehicle is driving straight and keeping its velocity at a constant level.
Two other cars are cutting in front of the ego vehicle from different adjacent lanes,
creating a 'squeeze' or 'double cut-in' maneuver.
Additionally, background traffic is added to increase realism and limit evasive options.

The ego vehicle will need to react to multiple threats, potentially braking aggressively
and making difficult decisions to avoid collisions.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      LaneChange,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp,
                                                                      KeepVelocity)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, DriveDistance, StandStill
from srunner.scenarios.basic_scenario import BasicScenario


class ComplexCutIn(BasicScenario):

    """
    The ego vehicle is driving on a highway and multiple cars are cutting in just in front,
    from different adjacent lanes. This is a single ego vehicle scenario with enhanced complexity.
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        # Base parameters for the cutting-in vehicles
        self._velocity = 40  # Base velocity for cutting vehicles
        self._delta_velocity_main_cutter = 10  # Acceleration delta for main cutter
        self._delta_velocity_secondary_cutter = 15  # Acceleration delta for secondary cutter (more aggressive)
        self._trigger_distance_main_cutter = 30 # Distance to ego for main cutter to start its move
        self._trigger_distance_secondary_cutter = 15 # Distance for secondary cutter to start its move (closer)

        # Parameters for main cutter (other_actors[0])
        self._main_cutter_direction = None
        self._transform_visible_main_cutter = None

        # Parameters for secondary cutter (other_actors[1])
        self._secondary_cutter_direction = None
        self._transform_visible_secondary_cutter = None

        # Parameters for background actor (if spawned)
        self._background_actor_index = -1
        self._transform_visible_background = None

        self._config = config # Storing config to get scenario name for direction

        super(ComplexCutIn, self).__init__("ComplexCutIn",
                                           ego_vehicles,
                                           config,
                                           world,
                                           debug_mode,
                                           criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(30, 70)
            self._trigger_distance_main_cutter = random.randint(20, 45)
            self._trigger_distance_secondary_cutter = random.randint(10, 25)
            self._delta_velocity_main_cutter = random.randint(8, 18)
            self._delta_velocity_secondary_cutter = random.randint(12, 22)


    def _initialize_actors(self, config):
        # Determine direction for the primary cutter (other_actors[0])
        if 'LEFT' in self._config.name.upper():
            self._main_cutter_direction = 'left'
        else:  # Default to right if not specified or 'RIGHT'
            self._main_cutter_direction = 'right'

        # Determine direction for the secondary cutter (other_actors[1]) - opposite of primary
        if self._main_cutter_direction == 'left':
            self._secondary_cutter_direction = 'right'
        else:
            self._secondary_cutter_direction = 'left'

        # We expect at least two 'other_actor' definitions in the scenario XML for the cutting vehicles.
        # Additional actors in config.other_actors will be treated as background traffic.
        num_expected_cutters = 2
        if len(config.other_actors) < num_expected_cutters:
            raise ValueError(f"ComplexCutIn scenario requires at least {num_expected_cutters} other_actor definitions in the config, but got {len(config.other_actors)}")

        self.other_actors = []
        for i, actor_config in enumerate(config.other_actors):
            vehicle = CarlaDataProvider.request_new_actor(actor_config.model, actor_config.transform)
            if vehicle is None:
                raise Exception(f"Failed to spawn actor {actor_config.model} at {actor_config.transform}")
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

            # Store visible transforms for each cutting vehicle.
            # They are spawned high above initially and then lowered by ActorTransformSetter.
            visible_transform = carla.Transform(
                carla.Location(actor_config.transform.location.x,
                               actor_config.transform.location.y,
                               actor_config.transform.location.z + 105), # Lifted high above
                actor_config.transform.rotation)

            if i == 0:
                self._transform_visible_main_cutter = visible_transform
            elif i == 1:
                self._transform_visible_secondary_cutter = visible_transform
            else:
                # Any additional actors beyond the first two are treated as background traffic.
                # Place them on a different lane, slightly offset if needed.
                ego_wp = self._map.get_waypoint(self.ego_vehicles[0].get_location())
                background_wp = ego_wp
                if ego_wp.get_right_lane() and self._main_cutter_direction != 'right':
                    background_wp = ego_wp.get_right_lane()
                elif ego_wp.get_left_lane() and self._main_cutter_direction != 'left':
                    background_wp = ego_wp.get_left_lane()
                elif ego_wp.get_right_lane(): # Fallback
                    background_wp = ego_wp.get_right_lane()
                else:
                    background_wp = ego_wp.next(50)[0] # Just further down ego lane if no other lane

                background_actor_transform = carla.Transform(
                    carla.Location(background_wp.transform.location.x + 80, # Spawn further ahead
                                   background_wp.transform.location.y,
                                   background_wp.transform.location.z + 105), # Lifted
                    background_wp.transform.rotation)

                background_vehicle = CarlaDataProvider.request_new_actor(actor_config.model, background_actor_transform)
                if background_vehicle:
                    self.other_actors.append(background_vehicle)
                    background_vehicle.set_simulate_physics(enabled=False)
                    self._transform_visible_background = background_actor_transform
                    self._background_actor_index = len(self.other_actors) - 1
                else:
                    self._background_actor_index = -1 # Indicate no background actor spawned


    def _create_behavior(self):
        """
        Order of sequence for each cutting vehicle:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """
        root = py_trees.composites.Sequence("ComplexCutInScenario")

        # Behavior for the main cutting vehicle (other_actors[0])
        main_cutter_behavior = py_trees.composites.Sequence("MainCutterSequence")
        main_cutter_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible_main_cutter, physics_id=0)
        main_cutter_behavior.add_child(main_cutter_visible)

        main_cutter_drive_until_trigger = py_trees.composites.Parallel(
            "MainCutterDrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        main_cutter_driving = WaypointFollower(self.other_actors[0], self._velocity)
        main_cutter_drive_until_trigger.add_child(main_cutter_driving)
        main_cutter_trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance_main_cutter)
        main_cutter_drive_until_trigger.add_child(main_cutter_trigger_distance)
        main_cutter_behavior.add_child(main_cutter_drive_until_trigger)

        main_cutter_accelerate = AccelerateToCatchUp(self.other_actors[0], self.ego_vehicles[0], throttle_value=1.0,
                                                     delta_velocity=self._delta_velocity_main_cutter,
                                                     trigger_distance=5, max_distance=500)
        main_cutter_behavior.add_child(main_cutter_accelerate)

        # Lane change direction is opposite to the direction it came from
        main_cutter_lane_change = LaneChange(
            self.other_actors[0], speed=None, direction='right' if self._main_cutter_direction == 'left' else 'left',
            distance_same_lane=5, distance_other_lane=300)
        main_cutter_behavior.add_child(main_cutter_lane_change)


        # Behavior for the secondary cutting vehicle (other_actors[1])
        secondary_cutter_behavior = py_trees.composites.Sequence("SecondaryCutterSequence")
        secondary_cutter_visible = ActorTransformSetter(self.other_actors[1], self._transform_visible_secondary_cutter, physics_id=0)
        secondary_cutter_behavior.add_child(secondary_cutter_visible)

        # Make the secondary cutter wait slightly before acting, to sequence the cut-ins.
        # It waits until the main cutter is closer to the ego.
        secondary_cutter_start_condition = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance_main_cutter - 10) # 10m closer than first one
        secondary_cutter_behavior.add_child(secondary_cutter_start_condition)

        secondary_cutter_drive_until_trigger = py_trees.composites.Parallel(
            "SecondaryCutterDrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        secondary_cutter_driving = WaypointFollower(self.other_actors[1], self._velocity + 5) # Slightly faster initial speed
        secondary_cutter_drive_until_trigger.add_child(secondary_cutter_driving)
        secondary_cutter_trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[1], self.ego_vehicles[0], self._trigger_distance_secondary_cutter) # Closer trigger distance
        secondary_cutter_drive_until_trigger.add_child(secondary_cutter_trigger_distance)
        secondary_cutter_behavior.add_child(secondary_cutter_drive_until_trigger)

        secondary_cutter_accelerate = AccelerateToCatchUp(self.other_actors[1], self.ego_vehicles[0], throttle_value=1.0,
                                                          delta_velocity=self._delta_velocity_secondary_cutter,
                                                          trigger_distance=3, max_distance=300) # More aggressive acceleration
        secondary_cutter_behavior.add_child(secondary_cutter_accelerate)

        secondary_cutter_lane_change = LaneChange(
            self.other_actors[1], speed=None, direction='right' if self._secondary_cutter_direction == 'left' else 'left',
            distance_same_lane=3, distance_other_lane=200) # Faster lane change parameters
        secondary_cutter_behavior.add_child(secondary_cutter_lane_change)


        # Behavior for background vehicle (if spawned)
        background_actor_behavior = py_trees.composites.Sequence("BackgroundActorSequence")
        if self._background_actor_index != -1:
            background_actor = self.other_actors[self._background_actor_index]
            background_visible = ActorTransformSetter(background_actor, self._transform_visible_background, physics_id=0)
            background_actor_behavior.add_child(background_visible)
            # This actor just drives straight at a slightly lower speed
            background_driving = WaypointFollower(background_actor, self._velocity - 5)
            background_actor_behavior.add_child(background_driving)
            # Add a standstill condition as well to ensure it's not blocking the end condition
            background_actor_behavior.add_child(StandStill(background_actor, name="Wait_For_End"))


        # Main parallel composite for all actor behaviors.
        # The primary and secondary cut-ins should happen somewhat concurrently but triggered sequentially.
        # The background actor just drives.
        all_actors_behavior = py_trees.composites.Parallel(
            "AllActorBehaviors", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        all_actors_behavior.add_child(main_cutter_behavior)
        all_actors_behavior.add_child(secondary_cutter_behavior)
        if self._background_actor_index != -1:
            all_actors_behavior.add_child(background_actor_behavior)

        # End condition for the entire scenario: ego drives a defined distance.
        # This condition should ideally be for the ego vehicle, or after the critical phase.
        endcondition = DriveDistance(self.ego_vehicles[0], 350) # Increased distance for ego to drive

        # Build the overall behavior tree
        root.add_child(all_actors_behavior)
        root.add_child(endcondition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        # The primary criteria for safety in this scenario is to avoid collision
        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors after deletion.
        """
        self.remove_all_actors()
```