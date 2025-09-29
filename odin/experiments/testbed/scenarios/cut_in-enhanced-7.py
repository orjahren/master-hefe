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

Enhanced Scenario:
This version of the CutIn scenario increases complexity by adding multiple
additional traffic vehicles on the highway. These vehicles will also drive
forward at varying speeds, creating a more dynamic and challenging environment
for the ego vehicle to manage the cut-in maneuver.
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
    This is a single ego vehicle scenario, enhanced with additional traffic.
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

        # New variables for added complexity
        self._num_additional_traffic_cars = 0 # Default value
        self._traffic_car_models = ['vehicle.audi.a2', 'vehicle.lincoln.mkz_2017',
                                    'vehicle.nissan.patrol', 'vehicle.mercedes.coupe',
                                    'vehicle.tesla.cybertruck', 'vehicle.volkswagen.t2'] # Diverse car models

        super(CutIn, self).__init__("CutIn",
                                    ego_vehicles,
                                    config,
                                    world,
                                    debug_mode,
                                    criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)
            self._num_additional_traffic_cars = random.randint(2, 5) # Randomize number of extra cars


    def _initialize_actors(self, config):

        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add the main cut-in actor from xml file
        if config.other_actors: # Ensure there's at least one actor in config for the cutting car
            main_cut_in_actor_config = config.other_actors[0]
            vehicle = CarlaDataProvider.request_new_actor(main_cut_in_actor_config.model, main_cut_in_actor_config.transform)
            self.other_actors.append(vehicle)
            # This actor will be initially placed high above the road, with physics disabled.
            # Physics will be enabled when its WaypointFollower behavior starts.
            vehicle.set_simulate_physics(enabled=False)
        else:
            raise ValueError("No other_actors defined in the scenario config for CutIn scenario, "
                             "at least one is required for the main cutting car.")

        self._cutting_car = self.other_actors[0] # Reference to the main cutting car

        # Define the 'visible' transform (high above the road) for the cutting car
        # We use the initial transform from the config for calculation, not the actor's current transform
        other_actor_initial_transform = config.other_actors[0].transform
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_initial_transform.location.x,
                           other_actor_initial_transform.location.y,
                           other_actor_initial_transform.location.z + 105), # 105m above road
            other_actor_initial_transform.rotation)

        # Determine reference waypoints for spawning additional traffic
        ego_wp = self._map.get_waypoint(self.ego_vehicles[0].get_transform().location)
        # Use the initial spawn point from config for the cutting car, not the visible one
        cutting_car_initial_wp = self._map.get_waypoint(config.other_actors[0].transform.location)

        # Ensure _num_additional_traffic_cars is set even if randomize was False
        if self._num_additional_traffic_cars == 0:
            self._num_additional_traffic_cars = random.randint(2, 4) # Default number of extra cars

        # Add additional traffic cars to increase scene complexity
        for i in range(self._num_additional_traffic_cars):
            spawn_wp = None
            car_model = random.choice(self._traffic_car_models)
            
            # Distribute cars strategically around the ego and cutting car
            if i == 0: # Car behind the cutting car on its initial lane
                spawn_wp = cutting_car_initial_wp.previous(random.uniform(25, 45))[0]
            elif i == 1: # Car ahead of Ego, in Ego's lane
                spawn_wp = ego_wp.next(random.uniform(40, 70))[0]
            elif i == 2: # Car behind Ego, in Ego's lane
                spawn_wp = ego_wp.previous(random.uniform(40, 60))[0]
            elif i == 3: # Car on the *other* adjacent lane, near ego
                other_lane_wp = None
                if self._direction == 'left': # Cutting car is on left lane, ego is middle. 'Other' is right lane.
                    if ego_wp.get_right_lane():
                        other_lane_wp = ego_wp.get_right_lane()
                elif self._direction == 'right': # Cutting car is on right lane, ego is middle. 'Other' is left lane.
                    if ego_wp.get_left_lane():
                        other_lane_wp = ego_wp.get_left_lane()
                
                if other_lane_wp:
                    # Spawn slightly ahead or behind ego on the other lane
                    spawn_wp = other_lane_wp.next(random.uniform(-20, 50))[0]
            elif i == 4: # Another car further down ego's lane
                 spawn_wp = ego_wp.next(random.uniform(80, 120))[0]


            if spawn_wp:
                # Ensure the waypoint is on a drivable lane (Driving or Shoulder)
                if spawn_wp.lane_type in [carla.LaneType.Driving, carla.LaneType.Shoulder]:
                    # Add a small vertical offset to prevent spawning exactly on the ground,
                    # which can sometimes lead to minor collisions upon physics activation.
                    spawn_transform = carla.Transform(
                        carla.Location(spawn_wp.transform.location.x,
                                       spawn_wp.transform.location.y,
                                       spawn_wp.transform.location.z + 0.1), # Small Z offset
                        spawn_wp.transform.rotation)
                    
                    traffic_car = CarlaDataProvider.request_new_actor(car_model, spawn_transform)
                    if traffic_car: # Make sure actor was successfully spawned
                        self.other_actors.append(traffic_car)
                        # Physics for these traffic cars will be enabled by their WaypointFollower behavior
                    else:
                        print(f"Warning: Failed to spawn traffic car {i+1} (model: {car_model}) at {spawn_transform.location}.")
                else:
                    print(f"Warning: Spawn waypoint {spawn_wp.transform.location} for traffic car {i+1} is not on a drivable lane ({spawn_wp.lane_type}). Skipping.")
            else:
                print(f"Warning: Could not determine valid spawn waypoint for traffic car {i+1}. Skipping.")


    def _create_behavior(self):
        """
        Behavior tree for the enhanced CutIn scenario.
        Order of sequence for the cutting car remains:
        - car_visible: spawn car at a visible transform (above road, physics OFF)
        - just_drive: drive until in trigger distance to ego_vehicle (physics ON)
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        
        Additional traffic cars drive continuously in parallel to the main event.
        """

        # 1. Main Cutting Car Behavior Sequence
        cutting_car_behavior_sequence = py_trees.composites.Sequence("CutInCarBehavior_{}".format(self._direction))

        # car_visible: spawn car at a visible transform high above the road, physics disabled
        # Physics will be enabled when WaypointFollower (or LaneChange) starts
        car_visible = ActorTransformSetter(self._cutting_car, self._transform_visible, physics_enabled=False)
        cutting_car_behavior_sequence.add_child(car_visible)

        # just_drive: drive until in trigger distance to ego_vehicle
        just_drive = py_trees.composites.Parallel(
            "DrivingStraightToTrigger", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # WaypointFollower will implicitly enable physics for the cutting car
        car_driving = WaypointFollower(self._cutting_car, self._velocity)
        just_drive.add_child(car_driving)
        trigger_distance = InTriggerDistanceToVehicle(
            self._cutting_car, self.ego_vehicles[0], self._trigger_distance)
        just_drive.add_child(trigger_distance)
        cutting_car_behavior_sequence.add_child(just_drive)

        # accelerate: accelerate to catch up distance to ego_vehicle
        accelerate = AccelerateToCatchUp(self._cutting_car, self.ego_vehicles[0], throttle_value=1,
                                         delta_velocity=self._delta_velocity, trigger_distance=5, max_distance=500)
        cutting_car_behavior_sequence.add_child(accelerate)

        # lane_change: change the lane
        if self._direction == 'left':
            lane_change = LaneChange(
                self._cutting_car, speed=None, direction='right', distance_same_lane=5, distance_other_lane=300)
            cutting_car_behavior_sequence.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self._cutting_car, speed=None, direction='left', distance_same_lane=5, distance_other_lane=300)
            cutting_car_behavior_sequence.add_child(lane_change)

        # 2. Additional Traffic Flow Behaviors
        traffic_flow_behaviors = py_trees.composites.Parallel(
            "TrafficFlow", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL) # All traffic cars should continue driving

        # Iterate from the second actor onwards (index 1), as other_actors[0] is the main cutting car
        for i in range(1, len(self.other_actors)):
            traffic_actor = self.other_actors[i]
            # Randomize speed for traffic cars, relative to ego velocity
            traffic_speed = self._velocity + random.uniform(-15, 15)
            if traffic_speed < 10: traffic_speed = 10 # Ensure a minimum speed
            traffic_flow_behaviors.add_child(WaypointFollower(traffic_actor, traffic_speed))

        # 3. Combine the cutting car's behavior and the continuous traffic flow
        # This parallel node succeeds as soon as one of its children succeeds.
        # This means the scenario will move to the end condition once the
        # cutting_car_behavior_sequence has completed its maneuver.
        main_scenario_elements = py_trees.composites.Parallel(
            "MainScenarioElementsAndTraffic", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        main_scenario_elements.add_child(cutting_car_behavior_sequence)
        
        # Only add traffic flow behaviors if there are actual additional traffic cars
        if len(self.other_actors) > 1:
            main_scenario_elements.add_child(traffic_flow_behaviors)

        # 4. Overall scenario root: a sequence to execute main events then the end condition
        root = py_trees.composites.Sequence("ScenarioBehaviorRoot", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(main_scenario_elements)

        # endcondition: The scenario concludes after the cutting car has driven an additional 200m
        # post-lane change. This condition is evaluated after main_scenario_elements has succeeded.
        endcondition = DriveDistance(self._cutting_car, 200)
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