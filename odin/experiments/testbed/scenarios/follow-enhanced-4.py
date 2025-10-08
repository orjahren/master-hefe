```python
#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Follow leading vehicle scenario:

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle follows a leading car driving down
a given road. At some point the leading car has to slow down and
finally stop. The ego vehicle has to react accordingly to avoid
a collision. The scenario ends either via a timeout, or if the ego
vehicle stopped close enough to the leading vehicle
"""

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class FollowLeadingVehicle(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicle, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        transform = waypoint.transform
        transform.location.z += 0.5
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', transform)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # let the other actor drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))

        # stop vehicle
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowLeadingVehicleWithObstacle(BasicScenario):

    """
    This class holds a scenario similar to FollowLeadingVehicle
    but there is an obstacle in front of the leading vehicle.
    Enhanced to include an additional static obstacle after the first,
    increasing complexity and decreasing driveability.

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        """
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._first_actor_location = 25
        self._second_actor_location = self._first_actor_location + 41  # Bicycle initial position
        self._third_actor_location = self._second_actor_location + 20  # New static obstacle position, 20m after bicycle's initial pos
        self._first_actor_speed = 10
        self._second_actor_speed = 1.5  # Bicycle speed
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._first_actor_transform = None
        self._second_actor_transform = None
        self._third_actor_transform = None  # Added for the new static obstacle

        super(FollowLeadingVehicleWithObstacle, self).__init__("FollowLeadingVehicleWithObstacle",
                                                               ego_vehicles,
                                                               config,
                                                               world,
                                                               debug_mode,
                                                               criteria_enable=criteria_enable)
        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # Leading Vehicle (first_actor)
        first_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z - 500),  # Initially hidden
            first_actor_waypoint.transform.rotation)
        self._first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z + 1),  # Actual spawn height
            first_actor_waypoint.transform.rotation)
        first_actor = CarlaDataProvider.request_new_actor(
            'vehicle.nissan.patrol', first_actor_transform)
        first_actor.set_simulate_physics(enabled=False)
        self.other_actors.append(first_actor)

        # Bicycle (second_actor)
        second_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)
        yaw_2 = second_actor_waypoint.transform.rotation.yaw + 90  # Original scenario has bicycle perpendicular to road
        second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z - 500),  # Initially hidden
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_2,
                           second_actor_waypoint.transform.rotation.roll))
        self._second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z + 1),  # Actual spawn height
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_2,
                           second_actor_waypoint.transform.rotation.roll))
        second_actor = CarlaDataProvider.request_new_actor(
            'vehicle.diamondback.century', second_actor_transform)
        second_actor.set_simulate_physics(enabled=False)
        self.other_actors.append(second_actor)

        # New Static Obstacle (third_actor)
        third_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._third_actor_location)
        yaw_3 = third_actor_waypoint.transform.rotation.yaw
        third_actor_transform_initial = carla.Transform(
            carla.Location(third_actor_waypoint.transform.location.x,
                           third_actor_waypoint.transform.location.y,
                           third_actor_waypoint.transform.location.z - 500),  # Hidden initially
            third_actor_waypoint.transform.rotation)

        self._third_actor_transform = carla.Transform(
            carla.Location(third_actor_waypoint.transform.location.x,
                           third_actor_waypoint.transform.location.y,
                           third_actor_waypoint.transform.location.z + 1),
            carla.Rotation(third_actor_waypoint.transform.rotation.pitch, yaw_3 + 15,  # Slightly angled
                           third_actor_waypoint.transform.rotation.roll))

        third_actor = CarlaDataProvider.request_new_actor(
            'vehicle.carlamotors.carlacola', third_actor_transform_initial)  # A large truck as a static obstacle
        third_actor.set_simulate_physics(enabled=False)  # It should remain static
        self.other_actors.append(third_actor)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive towards obstacles.
        First, the leading vehicle encounters a bicycle, which clears the road.
        Then, the leading vehicle encounters a static truck obstacle and stops,
        forcing the ego vehicle to react to multiple sequential stops.
        If this does not happen within the timeout, the scenario stops.
        """

        # Set initial transforms for all actors
        set_transform_actors = py_trees.composites.Parallel(
            "Set Actors Transform",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        set_transform_actors.add_child(ActorTransformSetter(self.other_actors[0], self._first_actor_transform))
        set_transform_actors.add_child(ActorTransformSetter(self.other_actors[1], self._second_actor_transform))
        set_transform_actors.add_child(ActorTransformSetter(self.other_actors[2], self._third_actor_transform))  # Set static obstacle

        # 1. Leading vehicle drives until reaching the bicycle obstacle
        driving_to_bicycle_obstacle = py_trees.composites.Parallel(
            "Driving towards Bicycle Obstacle",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_to_bicycle_obstacle.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_bicycle_obstacle.add_child(InTriggerDistanceToVehicle(self.other_actors[1],  # bicycle
                                                                          self.other_actors[0],  # leading vehicle
                                                                          distance=15))  # Stop 15m before bicycle

        # 2. Leading vehicle stops for the bicycle
        stop_for_bicycle = StopVehicle(self.other_actors[0], self._other_actor_max_brake, name="StopForBicycle")

        # 3. Wait for a moment and bicycle clears the road
        wait_and_bicycle_clears = py_trees.composites.Sequence("WaitAndBicycleClears")
        wait_and_bicycle_clears.add_child(TimeOut(3))  # Wait a bit
        obstacle_clear_road = py_trees.composites.Parallel("Bicycle Clearing Road",
                                                           policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_clear_road.add_child(DriveDistance(self.other_actors[1], 8))  # Bicycle moves a bit more to clear
        obstacle_clear_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))
        wait_and_bicycle_clears.add_child(obstacle_clear_road)
        wait_and_bicycle_clears.add_child(ActorDestroy(self.other_actors[1]))  # Bicycle gets removed after clearing

        # 4. Leading vehicle drives towards the new static obstacle (third_actor)
        driving_to_static_obstacle = py_trees.composites.Parallel(
            "Driving towards Static Obstacle",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_to_static_obstacle.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_static_obstacle.add_child(InTriggerDistanceToVehicle(self.other_actors[2],  # static obstacle (truck)
                                                                         self.other_actors[0],  # leading vehicle
                                                                         distance=10))  # Stop 10m before static obstacle

        # 5. Leading vehicle stops for the static obstacle (and stays stopped as it's static)
        stop_for_static_obstacle = StopVehicle(self.other_actors[0], self._other_actor_max_brake, name="StopForStaticObstacle")

        # End condition: ego stops behind the leading vehicle (which is now stopped by the static obstacle)
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,  # Ego needs to be within 20m of leading car
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)  # Ego must be stopped
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(set_transform_actors)
        sequence.add_child(driving_to_bicycle_obstacle)
        sequence.add_child(stop_for_bicycle)
        sequence.add_child(wait_and_bicycle_clears)
        sequence.add_child(driving_to_static_obstacle)  # New phase: leading vehicle drives to static obstacle
        sequence.add_child(stop_for_static_obstacle)  # New stop: leading vehicle stops for static obstacle
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))  # Destroy leading vehicle
        # other_actors[1] (bicycle) is destroyed earlier in 'wait_and_bicycle_clears'
        sequence.add_child(ActorDestroy(self.other_actors[2]))  # Destroy static obstacle (truck)

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
```