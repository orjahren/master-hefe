```python
#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Complex Follow leading vehicle scenario:

The scenario enhances the common driving behavior where the user-controlled ego
vehicle follows a leading car. This version adds several complexities to
decrease driveability:
1. Adverse weather conditions (rain, fog, wetness).
2. A parked car partially blocking the leading vehicle's lane, forcing it to perform a lane change.
3. A pedestrian crossing the road, forcing the leading vehicle to slow down and stop.

The ego vehicle has to react accordingly to avoid a collision, navigate the
complex interactions, and finally stop close enough to the leading vehicle.
"""

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      LaneChange,
                                                                      DriveDistance)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class FollowLeadingVehicleComplex(BasicScenario):

    """
    This class holds everything required for a complex "Follow a leading vehicle"
    scenario involving multiple dynamic and static obstacles, and adverse weather.
    (Traffic Scenario 2 enhanced)

    This is a single ego vehicle scenario
    """

    timeout = 180            # Increased Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25 # Initial distance of leading vehicle from ego's trigger
        self._parked_car_distance = 60    # Distance from ego's trigger to parked car
        self._pedestrian_spawn_distance = 90 # Distance from ego's trigger to pedestrian spawn
        
        self._first_vehicle_speed = 15 # Increased leading vehicle initial speed
        self._pedestrian_speed = 1.0 # Pedestrian crossing speed (m/s)

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._first_actor_transform = None # Store leading vehicle's initial transform
        
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicleComplex, self).__init__("FollowVehicleComplex",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)
            self._first_vehicle_speed = random.randint(10, 20)
            self._parked_car_distance = random.randint(50, 80)
            self._pedestrian_spawn_distance = random.randint(80, 120)

        # Set adverse weather conditions to decrease driveability
        world.set_weather(carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=80.0,
            precipitation_deposits=80.0,
            wind_intensity=30.0,
            fog_density=50.0,
            wetness=50.0 # Road wetness impacts friction
        ))


    def _initialize_actors(self, config):
        """
        Custom initialization of actors: leading vehicle, parked car, pedestrian.
        """
        # Leading Vehicle (self.other_actors[0])
        waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        self._first_actor_transform = waypoint.transform
        self._first_actor_transform.location.z += 0.5 # Ensure vehicle is slightly above ground
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', self._first_actor_transform)
        self.other_actors.append(first_vehicle)

        # Parked Car (self.other_actors[1])
        parked_car_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._parked_car_distance)
        # Offset to partially block the lane. Get right vector to offset relative to vehicle's forward direction.
        parked_car_transform = carla.Transform(
            parked_car_waypoint.transform.location + parked_car_waypoint.transform.get_right_vector() * 1.5,
            parked_car_waypoint.transform.rotation
        )
        parked_car_transform.location.z += 0.5 # Ensure it's on the ground
        parked_car = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', parked_car_transform)
        self.other_actors.append(parked_car)
        # Parked car should not move and physics should be off to ensure it stays in place
        parked_car.set_autopilot(False)
        parked_car.set_simulate_physics(False)

        # Pedestrian (self.other_actors[2])
        pedestrian_spawn_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._pedestrian_spawn_distance)
        # Place pedestrian on the sidewalk, facing to cross the road
        pedestrian_transform = carla.Transform(
            pedestrian_spawn_waypoint.transform.location + pedestrian_spawn_waypoint.transform.get_right_vector() * 4.0, # 4m to the side (sidewalk)
            pedestrian_spawn_waypoint.transform.rotation
        )
        # Adjust rotation to face towards the road (90 degrees left relative to current forward direction of the road)
        pedestrian_transform.rotation.yaw -= 90.0 # This might need adjustment based on map orientation
        pedestrian_transform.location.z += 0.5 # Ensure on ground
        
        # Request both the walker and its controller
        pedestrian = CarlaDataProvider.request_new_actor('walker.pedestrian.0001', pedestrian_transform, 'walker.controller.0001')
        self.other_actors.append(pedestrian)
        # Pedestrian should initially not simulate physics until it starts moving in the behavior tree
        pedestrian.set_simulate_physics(False)


    def _create_behavior(self):
        """
        The scenario behavior tree:
        1. Initialize actors to their starting positions.
        2. Leading vehicle drives towards a parked car and performs a lane change to avoid it.
        3. Leading vehicle encounters a pedestrian crossing the road, stops, waits for it to clear.
        4. Leading vehicle resumes driving towards the next intersection and stops.
        5. Ego vehicle must follow and also stop.
        """

        # Actors: self.other_actors[0] = leading_vehicle, self.other_actors[1] = parked_car, self.other_actors[2] = pedestrian

        # 1. Initial setup: Actors placed
        initial_actors_placement = py_trees.composites.Parallel("Initial Actor Placement", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        initial_actors_placement.add_child(ActorTransformSetter(self.other_actors[0], self._first_actor_transform))
        initial_actors_placement.add_child(ActorTransformSetter(self.other_actors[1], self.other_actors[1].get_transform()))
        initial_actors_placement.add_child(ActorTransformSetter(self.other_actors[2], self.other_actors[2].get_transform()))

        # 2. Leading vehicle approaches parked car and performs lane change
        lane_change_trigger_distance = 25 # Distance before parked car to initiate lane change
        
        # Behavior for leading vehicle driving towards parked car until trigger
        drive_to_parked_car_trigger = py_trees.composites.Parallel(
            "Drive to Parked Car Trigger",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        drive_to_parked_car_trigger.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        drive_to_parked_car_trigger.add_child(
            InTriggerDistanceToVehicle(self.other_actors[0], self.other_actors[1], distance=lane_change_trigger_distance))

        # Lane change behavior itself
        # After lane change, the vehicle will continue driving in the new lane.
        lane_change_avoid_parked = LaneChange(
            self.other_actors[0],
            direction='right', # Assuming a right lane change is available and safe on the map
            distance_same_lane=10, # How far to drive in current lane before starting change
            distance_other_lane=20 # How far to drive in other lane after changing
        )

        # 3. Leading vehicle approaches pedestrian and stops
        pedestrian_cross_distance = 5.0 # Pedestrian walks 5m across the road
        pedestrian_clear_road_time = pedestrian_cross_distance / self._pedestrian_speed # Time taken for pedestrian to cross
        
        # Pedestrian behavior: cross the road
        pedestrian_crossing_behavior = py_trees.composites.Sequence("Pedestrian Crossing Sequence")
        # Ensure pedestrian physics is enabled before it starts moving
        pedestrian_crossing_behavior.add_child(ActorTransformSetter(self.other_actors[2], self.other_actors[2].get_transform(), name="EnablePedPhysics", physics_id=True))
        pedestrian_crossing_behavior.add_child(KeepVelocity(self.other_actors[2], self._pedestrian_speed))
        pedestrian_crossing_behavior.add_child(DriveDistance(self.other_actors[2], target_distance=pedestrian_cross_distance))
        pedestrian_crossing_behavior.add_child(ActorDestroy(self.other_actors[2])) # Remove pedestrian after crossing

        # Leading vehicle drives until triggered to stop for pedestrian
        stop_for_pedestrian_trigger_distance = 15 # Leading vehicle stops within this distance of pedestrian
        leading_vehicle_stops_for_pedestrian_trigger = py_trees.composites.Parallel(
            "Leading Vehicle Stops for Pedestrian Trigger",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        leading_vehicle_stops_for_pedestrian_trigger.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        leading_vehicle_stops_for_pedestrian_trigger.add_child(
            InTriggerDistanceToVehicle(self.other_actors[0], self.other_actors[2], distance=stop_for_pedestrian_trigger_distance))

        # Sequence for handling pedestrian interaction:
        # 1. Lead vehicle drives until pedestrian is close
        # 2. Lead vehicle stops
        # 3. Pedestrian crosses while lead vehicle waits
        # 4. Lead vehicle resumes driving
        pedestrian_interaction = py_trees.composites.Sequence("Pedestrian Interaction")
        pedestrian_interaction.add_child(leading_vehicle_stops_for_pedestrian_trigger)
        pedestrian_interaction.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake, name="LeadVehicleStopForPedestrian"))
        
        # Pedestrian crosses while leading vehicle waits
        pedestrian_cross_and_wait = py_trees.composites.Parallel(
            "Pedestrian Crosses While Vehicle Waits",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL # Both children must succeed
        )
        pedestrian_cross_and_wait.add_child(TimeOut(pedestrian_clear_road_time + 1.0)) # Wait for pedestrian to cross + 1s buffer
        pedestrian_cross_and_wait.add_child(pedestrian_crossing_behavior)
        pedestrian_interaction.add_child(pedestrian_cross_and_wait)
        pedestrian_interaction.add_child(KeepVelocity(self.other_actors[0], self._first_vehicle_speed, name="LeadVehicleResume"))

        # 4. Leading vehicle drives to final intersection and stops (similar to original scenario)
        driving_to_next_intersection = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))

        stop_at_intersection = StopVehicle(self.other_actors[0], self._other_actor_max_brake, name="LeadVehicleStopAtIntersection")

        # 5. End condition: Ego vehicle must stop close to the leading vehicle
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20, # Distance threshold for ego to leading vehicle
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1) # Ego vehicle must be stationary
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build the complete behavior tree sequence
        sequence = py_trees.composites.Sequence("Complex Follow Leading Vehicle Behavior")
        sequence.add_child(initial_actors_placement)
        sequence.add_child(drive_to_parked_car_trigger)
        sequence.add_child(lane_change_avoid_parked)
        sequence.add_child(pedestrian_interaction)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop_at_intersection)
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0])) # Destroy leading vehicle
        sequence.add_child(ActorDestroy(self.other_actors[1])) # Destroy parked car
        # Pedestrian is destroyed within its own sequence

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        # The primary criterion is to avoid collision for the ego vehicle.
        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion and reset weather to default.
        """
        self.remove_all_actors()
        if self.world:
            self.world.set_weather(carla.WeatherParameters.Default) # Reset weather to default
```