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

        # Enhanced complexity attributes
        self._background_actors = []
        self._background_actor_speeds = []
        self._static_obstacle = None

        super(CutIn, self).__init__("CutIn",
                                    ego_vehicles,
                                    config,
                                    world,
                                    debug_mode,
                                    criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)
            # Make cut-in more aggressive sometimes
            self._delta_velocity = random.randint(10, 20) # Increased max delta_velocity for harder cut-in

        # Set complex weather conditions (rain, fog, dusk/night)
        self.world.set_weather(carla.WeatherParameters(
            cloudiness=random.uniform(60.0, 90.0),
            precipitation=random.uniform(30.0, 70.0),
            wetness=random.uniform(40.0, 80.0),
            fog_density=random.uniform(10.0, 30.0),
            sun_altitude_angle=random.uniform(-45.0, 15.0), # Low sun angle for dusk/dawn or negative for night
            wind_intensity=random.uniform(0.0, 20.0) # Add some wind
        ))


    def _initialize_actors(self, config):

        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add cut-in actor from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False) # Physics enabled later by WaypointFollower or AccelerateToCatchUp

        # transform visible for cut-in actor (spawned above road, then set to actual position)
        other_actor_transform = self.other_actors[0].get_transform()
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_transform.location.x,
                           other_actor_transform.location.y,
                           other_actor_transform.location.z + 105), # Spawn higher to be visible before descent
            other_actor_transform.rotation)

        # --- Add Background Traffic ---
        ego_initial_wp = self._map.get_waypoint(self.ego_vehicles[0].get_location(), lane_type=carla.LaneType.Driving)
        if ego_initial_wp is None: # Fallback if ego vehicle location is invalid at init
            ego_initial_wp = self._reference_waypoint

        # Background vehicle 1: Ahead on the same lane, slightly slower
        wp_bg1_ref = ego_initial_wp.next(random.uniform(80, 120))[0] if ego_initial_wp.next(1) else None
        if wp_bg1_ref:
            bg_speed_1 = self._velocity - random.uniform(5, 10)
            bg_actor_1 = CarlaDataProvider.request_new_actor('vehicle.carlamotors.carlacola', wp_bg1_ref.transform)
            if bg_actor_1:
                self._background_actors.append(bg_actor_1)
                self._background_actor_speeds.append(bg_speed_1)
                bg_actor_1.set_simulate_physics(True)

        # Background vehicle 2: Behind on the same lane, slightly faster
        wp_bg2_ref = ego_initial_wp.previous(random.uniform(50, 90))[0] if ego_initial_wp.previous(1) else None
        if wp_bg2_ref:
            bg_speed_2 = self._velocity + random.uniform(5, 10)
            bg_actor_2 = CarlaDataProvider.request_new_actor('vehicle.lincoln.mkz_2017', wp_bg2_ref.transform)
            if bg_actor_2:
                self._background_actors.append(bg_actor_2)
                self._background_actor_speeds.append(bg_speed_2)
                bg_actor_2.set_simulate_physics(True)

        # Background vehicle 3: On an adjacent lane, similar speed
        wp_bg3_ref = None
        if self._direction == 'left': # Cut-in comes from left, so traffic on right lane is relevant
            adj_lane_wp = ego_initial_wp.get_right_lane()
            if adj_lane_wp:
                wp_bg3_ref = adj_lane_wp.next(random.uniform(30, 70))[0] if adj_lane_wp.next(1) else None
        else: # Cut-in comes from right, so traffic on left lane is relevant
            adj_lane_wp = ego_initial_wp.get_left_lane()
            if adj_lane_wp:
                wp_bg3_ref = adj_lane_wp.next(random.uniform(30, 70))[0] if adj_lane_wp.next(1) else None

        if wp_bg3_ref:
            bg_speed_3 = self._velocity + random.uniform(-5, 5)
            bg_actor_3 = CarlaDataProvider.request_new_actor('vehicle.audi.a2', wp_bg3_ref.transform)
            if bg_actor_3:
                self._background_actors.append(bg_actor_3)
                self._background_actor_speeds.append(bg_speed_3)
                bg_actor_3.set_simulate_physics(True)


        # --- Add Static Obstacle (e.g., disabled car on shoulder) ---
        obstacle_wp_ref = ego_initial_wp.next(random.uniform(70, 100))[0] if ego_initial_wp.next(1) else None
        
        if obstacle_wp_ref:
            obstacle_transform = obstacle_wp_ref.transform
            # Attempt to find a shoulder lane or place it close to the right edge
            shoulder_wp = obstacle_wp_ref.get_right_lane()
            
            if shoulder_wp and shoulder_wp.lane_type == carla.LaneType.Shoulder:
                obstacle_transform = shoulder_wp.transform
                # Shift slightly off the road if it's too much on the lane
                right_vector = obstacle_transform.get_right_vector()
                obstacle_transform.location += right_vector * random.uniform(1.0, 2.0)
            else: # If no explicit right shoulder, place it slightly off the main lane
                right_vector = obstacle_transform.get_right_vector()
                obstacle_transform.location += right_vector * random.uniform(2.5, 3.5) # 2.5-3.5 meters off the lane to the right
            
            # Request a disabled car model
            self._static_obstacle = CarlaDataProvider.request_new_actor('vehicle.volkswagen.t2', obstacle_transform)
            if self._static_obstacle:
                self.other_actors.append(self._static_obstacle)
                self._static_obstacle.set_simulate_physics(False) # Keep it static


    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn cut-in car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """

        # Root behavior for the whole scenario (Parallel composition)
        # This allows multiple independent behaviors (cut-in, background traffic) to run concurrently.
        # Scenario ends when the main cut-in sequence finishes (SUCCESS_ON_ONE).
        root = py_trees.composites.Parallel("Complex Scenario Behavior",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # --- Cut-in actor behavior (existing logic) ---
        cut_in_behavior = py_trees.composites.Sequence("CutIn_Actor_Sequence")
        car_visible = ActorTransformSetter(self.other_actors[0], self._transform_visible)
        cut_in_behavior.add_child(car_visible)

        just_drive = py_trees.composites.Parallel(
            "DrivingStraightUntilTrigger", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)

        trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance)
        just_drive.add_child(trigger_distance)
        cut_in_behavior.add_child(just_drive)

        accelerate = AccelerateToCatchUp(self.other_actors[0], self.ego_vehicles[0], throttle_value=1,
                                         delta_velocity=self._delta_velocity, trigger_distance=5, max_distance=500)
        cut_in_behavior.add_child(accelerate)

        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[0], speed=None, direction='right', distance_same_lane=5, distance_other_lane=300)
            cut_in_behavior.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self.other_actors[0], speed=None, direction='left', distance_same_lane=5, distance_other_lane=300)
            cut_in_behavior.add_child(lane_change)

        endcondition = DriveDistance(self.other_actors[0], 200) # Cut-in actor drives for 200m after its maneuver
        cut_in_behavior.add_child(endcondition)

        root.add_child(cut_in_behavior) # Add the cut-in sequence as a child to the main parallel root

        # --- Add Background Traffic behaviors ---
        for i, bg_actor in enumerate(self._background_actors):
            if i < len(self._background_actor_speeds): # Ensure speed exists for this actor
                speed = self._background_actor_speeds[i]
                # Background vehicles follow waypoints and avoid collisions (low level planner)
                bg_behavior = WaypointFollower(bg_actor, speed, avoid_collision=True)
                root.add_child(bg_behavior)

        # The static obstacle is just positioned and doesn't need an active behavior node.

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        # The primary criteria is still avoiding collision for the ego vehicle.
        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors after deletion and reset weather.
        """
        # Remove static obstacle if it was spawned and is still alive
        if self._static_obstacle and self._static_obstacle.is_alive:
            CarlaDataProvider.remove_actor_by_id(self._static_obstacle.id)
        
        # Remove background actors
        for actor in self._background_actors:
            if actor and actor.is_alive: # Check if actor exists and is alive before removal
                CarlaDataProvider.remove_actor_by_id(actor.id)
        
        # This will remove other_actors (including the cut-in vehicle) and ego_vehicles
        self.remove_all_actors() 
        
        # Reset weather to a clear state after the scenario
        if self.world:
            self.world.set_weather(carla.WeatherParameters.ClearNoon)

```