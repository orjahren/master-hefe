```python
#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenarios in which another (opposite) vehicle 'illegally' takes
priority, e.g. by running a red traffic light.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      SwitchWrongDirectionTest,
                                                                      BasicAgentBehavior,
                                                                      ScenarioTimeout,
                                                                      Idle, WaitForever,
                                                                      HandBrakeVehicle,
                                                                      OppositeActorFlow)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (DriveDistance,
                                                                               InTriggerDistanceToLocation,
                                                                               InTriggerDistanceToVehicle,
                                                                               WaitUntilInFront,
                                                                               WaitUntilInFrontPosition)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.background_manager import LeaveSpaceInFront, SetMaxSpeed, ChangeOppositeBehavior, ChangeRoadBehavior


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default

def get_interval_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return [
            p_type(config.other_parameters[name]['from']),
            p_type(config.other_parameters[name]['to'])
        ]
    else:
        return default


class Accident(BasicScenario):
    """
    This class holds everything required for a scenario in which there is an accident
    in front of the ego, forcing it to lane change. A police vehicle is located before
    two other cars that have been in an accident.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        
        # Original distances
        self._first_distance = 10
        self._second_distance = 6
        
        # NEW: Third accident vehicle distance, increasing complexity
        self._third_distance = 6 

        self._trigger_distance = 50
        self._end_distance = 50
        self._wait_duration = 5
        self._offset = 0.6 # Original offset for accident vehicles

        self._lights = carla.VehicleLightState.Special1 | carla.VehicleLightState.Special2 | carla.VehicleLightState.Position

        self._distance = get_value_parameter(config, 'distance', float, 120)
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        self._max_speed = get_value_parameter(config, 'speed', float, 60)
        self._scenario_timeout = 240

        # NEW: Pedestrian parameters, enhancing dynamic complexity
        self._pedestrian_bps = [
            "walker.pedestrian.0001", "walker.pedestrian.0002", "walker.pedestrian.0003",
            "walker.pedestrian.0004", "walker.pedestrian.0005", "walker.pedestrian.0006",
            "walker.pedestrian.0007", "walker.pedestrian.0008", "walker.pedestrian.0009",
            "walker.pedestrian.0010", "walker.pedestrian.0011", "walker.pedestrian.0012",
            "walker.pedestrian.0013", "walker.pedestrian.0014"
        ]
        self._pedestrian_speed = get_value_parameter(config, 'pedestrian_speed', float, 1.5) # ~5.4 km/h
        self._pedestrian_drive_distance = get_value_parameter(config, 'pedestrian_drive_distance', float, 10)
        self._pedestrian_offset = 0.9 # Further to the side, potentially on sidewalk, for spawning
        self._pedestrian_trigger_distance = 20 # Smaller trigger for pedestrian start

        # NEW: Traffic cone parameters, enhancing static obstruction
        self._cone_offset = 0.8 # Slightly more off the lane than accident cars, but less than pedestrian

        super().__init__(
            "Accident", ego_vehicles, config, world, randomize, debug_mode, criteria_enable=criteria_enable)

    def _move_waypoint_forward(self, wp, distance):
        dist = 0
        next_wp = wp
        while dist < distance:
            next_wps = next_wp.next(1)
            if not next_wps or next_wps[0].is_junction:
                break
            next_wp = next_wps[0]
            dist += 1
        return next_wp

    def _spawn_side_prop(self, wp):
        # Spawn the accident indication signal
        prop_wp = wp
        while True:
            if self._direction == "right":
                wp = prop_wp.get_right_lane()
            else:
                wp = prop_wp.get_left_lane()
            if wp is None or wp.lane_type not in (carla.LaneType.Driving, carla.LaneType.Parking):
                break
            prop_wp = wp

        displacement = 0.3 * prop_wp.lane_width
        r_vec = prop_wp.transform.get_right_vector()
        if self._direction == 'left':
            r_vec *= -1

        spawn_transform = wp.transform
        spawn_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=0.2)
        spawn_transform.rotation.yaw += 90
        signal_prop = CarlaDataProvider.request_new_actor('static.prop.warningaccident', spawn_transform)
        if not signal_prop:
            raise ValueError("Couldn't spawn the indication prop asset")
        signal_prop.set_simulate_physics(False)
        self.other_actors.append(signal_prop)

    def _spawn_obstacle(self, wp, blueprint, accident_actor=False):
        """
        Spawns the obstacle actor by displacing its position to the right
        """
        displacement = self._offset * wp.lane_width / 2
        r_vec = wp.transform.get_right_vector()
        if self._direction == 'left':
            r_vec *= -1

        spawn_transform = wp.transform
        spawn_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=1)
        if accident_actor:
            actor = CarlaDataProvider.request_new_actor(
                blueprint, spawn_transform, rolename='scenario no lights', attribute_filter={'base_type': 'car', 'generation': 2})
        else:
            actor = CarlaDataProvider.request_new_actor(
                blueprint, spawn_transform, rolename='scenario')
        if not actor:
            raise ValueError("Couldn't spawn an obstacle actor")

        return actor

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        starting_wp = self._map.get_waypoint(config.trigger_points[0].location)

        # Spawn the accident indication signal
        self._spawn_side_prop(starting_wp)

        # Spawn the police vehicle
        self._accident_wp = self._move_waypoint_forward(starting_wp, self._distance)
        police_car = self._spawn_obstacle(self._accident_wp, 'vehicle.dodge.charger_police_2020')

        # Set its initial conditions
        lights = police_car.get_light_state()
        lights |= self._lights
        police_car.set_light_state(carla.VehicleLightState(lights))
        police_car.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(police_car)

        # Create the first vehicle that has been in the accident
        self._first_vehicle_wp = self._move_waypoint_forward(self._accident_wp, self._first_distance)
        first_actor = self._spawn_obstacle(self._first_vehicle_wp, 'vehicle.*', True)

        # Set its initial conditions
        first_actor.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(first_actor)

        # Create the second vehicle that has been in the accident
        second_vehicle_wp = self._move_waypoint_forward(self._first_vehicle_wp, self._second_distance)
        second_actor = self._spawn_obstacle(second_vehicle_wp, 'vehicle.*', True)

        # Set its initial conditions
        second_actor.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(second_actor)

        # NEW: Create a third vehicle that has been in the accident, increasing obstruction
        third_vehicle_wp = self._move_waypoint_forward(second_vehicle_wp, self._third_distance)
        third_actor = self._spawn_obstacle(third_vehicle_wp, 'vehicle.*', True)

        # Set its initial conditions
        third_actor.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(third_actor)

        self._accident_wp = third_vehicle_wp # Update the main accident waypoint to the last car
        self._end_wp = self._move_waypoint_forward(third_vehicle_wp, self._end_distance)

        # NEW: Spawn traffic cones around the accident, further narrowing the path
        cone_ref_wps = [
            self._move_waypoint_forward(self._accident_wp, -self._first_distance - self._second_distance - 2), # Before first car
            self._first_vehicle_wp,
            self._move_waypoint_forward(self._first_vehicle_wp, self._second_distance / 2), # In between first and second
            second_vehicle_wp,
            self._move_waypoint_forward(second_vehicle_wp, self._third_distance / 2), # In between second and third
            third_vehicle_wp,
            self._move_waypoint_forward(third_vehicle_wp, 2) # After third car
        ]

        for i, wp in enumerate(cone_ref_wps):
            displacement = self._cone_offset * wp.lane_width / 2
            r_vec = wp.transform.get_right_vector()
            if self._direction == 'left':
                r_vec *= -1
            cone_transform = wp.transform
            cone_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=0.1) # Cones on ground
            cone_blueprint = 'static.prop.construction.trafficcone01'
            cone_actor = CarlaDataProvider.request_new_actor(cone_blueprint, cone_transform)
            if not cone_actor:
                raise ValueError(f"Couldn't spawn cone {i}")
            cone_actor.set_simulate_physics(False)
            self.other_actors.append(cone_actor)

        # NEW: Spawn a pedestrian near the accident, adding dynamic interaction
        pedestrian_spawn_wp = self._move_waypoint_forward(self._first_vehicle_wp, self._second_distance / 2) # In between first and second accident cars
        displacement = self._pedestrian_offset * pedestrian_spawn_wp.lane_width / 2 # Further to the side, simulate sidewalk
        r_vec = pedestrian_spawn_wp.transform.get_right_vector()
        if self._direction == 'left':
            r_vec *= -1
        pedestrian_transform = pedestrian_spawn_wp.transform
        pedestrian_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=0.1) # Pedestrian on ground

        rng = CarlaDataProvider.get_random_seed()
        ped_blueprint = rng.choice(self._pedestrian_bps)
        pedestrian_actor = CarlaDataProvider.request_new_actor(ped_blueprint, pedestrian_transform)
        if not pedestrian_actor:
            raise ValueError("Couldn't spawn pedestrian actor")
        pedestrian_actor.set_simulate_physics(True) # Pedestrians are typically simulated
        self.other_actors.append(pedestrian_actor)

        # Define pedestrian target location (slightly moving towards the road to be more intrusive)
        ped_target_wp = self._move_waypoint_forward(pedestrian_spawn_wp, self._pedestrian_drive_distance)
        self._pedestrian_target_loc = ped_target_wp.transform.location
        # Shift target location slightly into the lane for more challenge
        if self._direction == 'right':
            self._pedestrian_target_loc.y -= 0.5 # Shift left (towards road center) for right-hand traffic
        else: # 'left'
            self._pedestrian_target_loc.y += 0.5 # Shift right (towards road center) for left-hand traffic

    def _create_behavior(self):
        """
        The vehicle has to drive to reach a specific point but an accident is in the middle of the road,
        blocking its route and forcing it to lane change. Enhanced with more complexity.
        """
        root = py_trees.composites.Sequence(name="Accident")
        if self.route_mode:
            # Update total_dist for LeaveSpaceInFront to include the new third vehicle
            total_dist = self._distance + self._first_distance + self._second_distance + self._third_distance + 20
            root.add_child(LeaveSpaceInFront(total_dist))

        # Main parallel behavior for the active part of the scenario
        main_behavior_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="Main Behavior Parallel")
        
        main_behavior_parallel.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        # 1. Ego-centric behavior (triggers speed/waiting)
        ego_trigger_behavior = py_trees.composites.Sequence(name="Ego Trigger Behavior")
        ego_trigger_behavior.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._first_vehicle_wp.transform.location, self._trigger_distance))
        ego_trigger_behavior.add_child(Idle(self._wait_duration))
        if self.route_mode:
            ego_trigger_behavior.add_child(SetMaxSpeed(self._max_speed))
        ego_trigger_behavior.add_child(WaitForever()) # This ensures ego behavior lasts until something else finishes the parallel

        main_behavior_parallel.add_child(ego_trigger_behavior)

        # 2. Pedestrian movement behavior (last actor added is the pedestrian)
        pedestrian_actor = self.other_actors[-1] 
        if pedestrian_actor.type_id.startswith('walker'): # Ensure it's actually a walker type
            pedestrian_behavior = py_trees.composites.Sequence(name="Pedestrian Movement")
            # Pedestrian starts moving when ego is closer to its location
            pedestrian_behavior.add_child(InTriggerDistanceToLocation(
                self.ego_vehicles[0], pedestrian_actor.get_location(), self._pedestrian_trigger_distance))
            # BasicAgentBehavior for the pedestrian
            pedestrian_behavior.add_child(BasicAgentBehavior(
                pedestrian_actor, self._pedestrian_target_loc, target_speed=self._pedestrian_speed))
            pedestrian_behavior.add_child(HandBrakeVehicle(pedestrian_actor, 1)) # Stop movement after reaching target (or collision)
            pedestrian_behavior.add_child(WaitForever()) # Keep it active, don't let it stop the parallel
            main_behavior_parallel.add_child(pedestrian_behavior)

        # 3. Scenario End Condition (ego reaches a point past the accident)
        scenario_end_condition = py_trees.composites.Sequence(name="Scenario End Condition")
        scenario_end_condition.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._end_wp.transform, False))
        main_behavior_parallel.add_child(scenario_end_condition)

        root.add_child(main_behavior_parallel)

        if self.route_mode:
            root.add_child(SetMaxSpeed(0))
        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = [ScenarioTimeoutTest(self.ego_vehicles[0], self.config.name)]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()


class AccidentTwoWays(Accident):
    """
    Variation of the Accident scenario but the ego now has to invade the opposite lane
    """
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=180):

        self._opposite_interval = get_interval_parameter(config, 'frequency', float, [20, 100])
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _create_behavior(self):
        """
        The vehicle has to drive the whole predetermined distance. Adapt the opposite flow to
        let the ego invade the opposite lane.
        """
        reference_wp = self._accident_wp.get_left_lane()
        if not reference_wp:
            raise ValueError("Couldnt find a left lane to spawn the opposite traffic")

        root = py_trees.composites.Sequence(name="AccidentTwoWays")
        if self.route_mode:
            total_dist = self._distance + self._first_distance + self._second_distance + 20
            root.add_child(LeaveSpaceInFront(total_dist))

        end_condition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        end_condition.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._end_wp.transform, False))

        behavior = py_trees.composites.Sequence()
        behavior.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._first_vehicle_wp.transform.location, self._trigger_distance))
        behavior.add_child(Idle(self._wait_duration))
        if self.route_mode:
            behavior.add_child(SwitchWrongDirectionTest(False))
            behavior.add_child(ChangeOppositeBehavior(active=False))
        behavior.add_child(OppositeActorFlow(reference_wp, self.ego_vehicles[0], self._opposite_interval))

        end_condition.add_child(behavior)
        root.add_child(end_condition)

        if self.route_mode:
            root.add_child(SwitchWrongDirectionTest(True))
            root.add_child(ChangeOppositeBehavior(active=True))
        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root

class ParkedObstacle(BasicScenario):
    """
    Scenarios in which a parked vehicle is incorrectly parked,
    forcing the ego to lane change out of the route's lane
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._trigger_distance = 50
        self._end_distance = 50
        self._wait_duration = 5
        self._offset = 0.7

        self._lights = carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker | carla.VehicleLightState.Position

        self._distance = get_value_parameter(config, 'distance', float, 120)
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        self._max_speed = get_value_parameter(config, 'speed', float, 60)
        self._scenario_timeout = 240

        super().__init__(
            "ParkedObstacle", ego_vehicles, config, world, randomize, debug_mode, criteria_enable=criteria_enable)

    def _move_waypoint_forward(self, wp, distance):
        dist = 0
        next_wp = wp
        while dist < distance:
            next_wps = next_wp.next(1)
            if not next_wps or next_wps[0].is_junction:
                break
            next_wp = next_wps[0]
            dist += 1
        return next_wp

    def _spawn_side_prop(self, wp):
        # Spawn the accident indication signal
        prop_wp = wp
        while True:
            if self._direction == "right":
                wp = prop_wp.get_right_lane()
            else:
                wp = prop_wp.get_left_lane()
            if wp is None or wp.lane_type not in (carla.LaneType.Driving, carla.LaneType.Parking):
                break
            prop_wp = wp

        displacement = 0.3 * prop_wp.lane_width
        r_vec = prop_wp.transform.get_right_vector()
        if self._direction == 'left':
            r_vec *= -1

        spawn_transform = wp.transform
        spawn_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=0.2)
        spawn_transform.rotation.yaw += 90
        signal_prop = CarlaDataProvider.request_new_actor('static.prop.warningaccident', spawn_transform)
        if not signal_prop:
            raise ValueError("Couldn't spawn the indication prop asset")
        signal_prop.set_simulate_physics(False)
        self.other_actors.append(signal_prop)

    def _spawn_obstacle(self, wp, blueprint):
        """
        Spawns the obstacle actor by displacing its position to the right
        """
        displacement = self._offset * wp.lane_width / 2
        r_vec = wp.transform.get_right_vector()
        if self._direction == 'left':
            r_vec *= -1

        spawn_transform = wp.transform
        spawn_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=1)
        actor = CarlaDataProvider.request_new_actor(
            blueprint, spawn_transform, rolename='scenario no lights', attribute_filter={'base_type': 'car', 'generation': 2})
        if not actor:
            raise ValueError("Couldn't spawn an obstacle actor")

        return actor

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._starting_wp = self._map.get_waypoint(config.trigger_points[0].location)

        # Create the side prop
        self._spawn_side_prop(self._starting_wp)

        # Create the first vehicle that has been in the accident
        self._vehicle_wp = self._move_waypoint_forward(self._starting_wp, self._distance)
        parked_actor = self._spawn_obstacle(self._vehicle_wp, 'vehicle.*')

        lights = parked_actor.get_light_state()
        lights |= self._lights
        parked_actor.set_light_state(carla.VehicleLightState(lights))
        parked_actor.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(parked_actor)

        self._end_wp = self._move_waypoint_forward(self._vehicle_wp, self._end_distance)

    def _create_behavior(self):
        """
        The vehicle has to drive the whole predetermined distance.
        """
        root = py_trees.composites.Sequence(name="ParkedObstacle")
        if self.route_mode:
            total_dist = self._distance + 20
            root.add_child(LeaveSpaceInFront(total_dist))

        end_condition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        end_condition.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._end_wp.transform, False))

        behavior = py_trees.composites.Sequence()
        behavior.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._vehicle_wp.transform.location, self._trigger_distance))
        behavior.add_child(Idle(self._wait_duration))
        if self.route_mode:
            behavior.add_child(SetMaxSpeed(self._max_speed))
        behavior.add_child(WaitForever())

        end_condition.add_child(behavior)
        root.add_child(end_condition)

        if self.route_mode:
            root.add_child(SetMaxSpeed(0))
        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = [ScenarioTimeoutTest(self.ego_vehicles[0], self.config.name)]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()


class ParkedObstacleTwoWays(ParkedObstacle):
    """
    Variation of the ParkedObstacle scenario but the ego now has to invade the opposite lane
    """
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=180):

        self._opposite_interval = get_interval_parameter(config, 'frequency', float, [20, 100])
        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _create_behavior(self):
        """
        The vehicle has to drive the whole predetermined distance. Adapt the opposite flow to
        let the ego invade the opposite lane.
        """
        reference_wp = self._vehicle_wp.get_left_lane()
        if not reference_wp:
            raise ValueError("Couldnt find a left lane to spawn the opposite traffic")

        root = py_trees.composites.Sequence(name="ParkedObstacleTwoWays")
        if self.route_mode:
            total_dist = self._distance + 20
            root.add_child(LeaveSpaceInFront(total_dist))

        end_condition = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        end_condition.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._end_wp.transform, False))

        behavior = py_trees.composites.Sequence()
        behavior.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], self._vehicle_wp.transform.location, self._trigger_distance))
        behavior.add_child(Idle(self._wait_duration))
        if self.route_mode:
            behavior.add_child(SwitchWrongDirectionTest(False))
            behavior.add_child(ChangeOppositeBehavior(active=False))
        behavior.add_child(OppositeActorFlow(reference_wp, self.ego_vehicles[0], self._opposite_interval))

        end_condition.add_child(behavior)
        root.add_child(end_condition)

        if self.route_mode:
            root.add_child(SwitchWrongDirectionTest(True))
            root.add_child(ChangeOppositeBehavior(active=True))
        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root


class HazardAtSideLane(BasicScenario):
    """
    Added the dangerous scene of ego vehicles driving on roads without sidewalks,
    with three bicycles encroaching on some roads in front.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._obstacle_distance = 9
        self._trigger_distance = 50
        self._end_distance = 50
        self._extra_space = 30

        self._offset = 0.55
        self._wait_duration = 5

        self._target_locs = []

        self._bicycle_bps = ["vehicle.bh.crossbike", "vehicle.diamondback.century", "vehicle.gazelle.omafiets"]

        self._distance = get_value_parameter(config, 'distance', float, 100)
        self._max_speed = get_value_parameter(config, 'speed', float, 60)
        self._bicycle_speed = get_value_parameter(config, 'bicycle_speed', float, 10)
        self._bicycle_drive_distance = get_value_parameter(config, 'bicycle_drive_distance', float, 50)
        self._scenario_timeout = 240

        super().__init__("HazardAtSideLane",
                         ego_vehicles,
                         config,
                         world,
                         randomize,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _move_waypoint_forward(self, wp, distance):
        dist = 0
        next_wp = wp
        while dist < distance:
            next_wps = next_wp.next(1)
            if not next_wps or next_wps[0].is_junction:
                break
            next_wp = next_wps[0]
            dist += 1
        return next_wp

    def _spawn_obstacle(self, wp, blueprint):
        """
        Spawns the obstacle actor by displacing its position to the right
        """
        displacement = self._offset * wp.lane_width / 2
        r_vec = wp.transform.get_right_vector()

        spawn_transform = wp.transform
        spawn_transform.location += carla.Location(x=displacement * r_vec.x, y=displacement * r_vec.y, z=1)
        actor = CarlaDataProvider.request_new_actor(blueprint, spawn_transform)
        if not actor:
            raise ValueError("Couldn't spawn an obstacle actor")

        return actor

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        rng = CarlaDataProvider.get_random_seed()
        self._starting_wp = self._map.get_waypoint(config.trigger_points[0].location)

        # Spawn the first bicycle
        first_wp = self._move_waypoint_forward(self._starting_wp, self._distance)
        bicycle_1 = self._spawn_obstacle(first_wp, rng.choice(self._bicycle_bps))

        wps = first_wp.next(self._bicycle_drive_distance)
        if not wps:
            raise ValueError("Couldn't find an end location for the bicycles")
        self._target_locs.append(wps[0].transform.location)

        # Set its initial conditions
        bicycle_1.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(bicycle_1)

        # Spawn the second bicycle
        second_wp = self._move_waypoint_forward(first_wp, self._obstacle_distance)
        bicycle_2 = self._spawn_obstacle(second_wp, rng.choice(self._bicycle_bps))

        wps = second_wp.next(self._bicycle_drive_distance)
        if not wps:
            raise ValueError("Couldn't find an end location for the bicycles")
        self._target_locs.append(wps[0].transform.location)

        # Set its initial conditions
        bicycle_2.apply_control(carla.VehicleControl(hand_brake=True))
        self.other_actors.append(bicycle_2)

    def _create_behavior(self):
        """
        Activate the bicycles and wait for the ego to be close-by before changing the side traffic.
        End condition is based on the ego behind in front of the bicycles, or timeout based.
        """
        root = py_trees.composites.Sequence(name="HazardAtSideLane")
        if self.route_mode:
            total_dist = self._distance + self._obstacle_distance + 20
            root.add_child(LeaveSpaceInFront(total_dist))
            root.add_child(ChangeRoadBehavior(extra_space=self._extra_space))

        main_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        main_behavior.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        # End condition
        end_condition = py_trees.composites.Sequence(name="End Condition")
        end_condition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[-1], check_distance=False))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        main_behavior.add_child(end_condition)

        # Bicycle movement. Move them for a set distance, then stop
        offset = self._offset * self._starting_wp.lane_width / 2
        opt_dict = {'offset': offset}
        for actor, target_loc in zip(self.other_actors, self._target_locs):
            bicycle = py_trees.composites.Sequence(name="Bicycle behavior")
            bicycle.add_child(BasicAgentBehavior(actor, target_loc, target_speed=self._bicycle_speed, opt_dict=opt_dict))
            bicycle.add_child(HandBrakeVehicle(actor, 1))  # In case of collisions
            bicycle.add_child(WaitForever())  # Don't make the bicycle stop the parallel behavior
            main_behavior.add_child(bicycle)

        behavior = py_trees.composites.Sequence(name="Side lane behavior")
        behavior.add_child(InTriggerDistanceToVehicle(
            self.ego_vehicles[0], self.other_actors[0], self._trigger_distance))
        behavior.add_child(Idle(self._wait_duration))
        if self.route_mode:
            behavior.add_child(SetMaxSpeed(self._max_speed))
        behavior.add_child(WaitForever())

        main_behavior.add_child(behavior)

        root.add_child(main_behavior)
        if self.route_mode:
            root.add_child(SetMaxSpeed(0))
            root.add_child(ChangeRoadBehavior(extra_space=0))

        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = [ScenarioTimeoutTest(self.ego_vehicles[0], self.config.name)]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()


class HazardAtSideLaneTwoWays(HazardAtSideLane):
    """
    Variation of the HazardAtSideLane scenario but the ego now has to invade the opposite lane
    """
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=180):

        self._opposite_frequency = get_value_parameter(config, 'frequency', float, 100)

        super().__init__(world, ego_vehicles, config, randomize, debug_mode, criteria_enable, timeout)

    def _create_behavior(self):
        """
        Activate the bicycles and wait for the ego to be close-by before changing the opposite traffic.
        End condition is based on the ego behind in front of the bicycles, or timeout based.
        """

        root = py_trees.composites.Sequence(name="HazardAtSideLaneTwoWays")
        if self.route_mode:
            total_dist = self._distance + self._obstacle_distance + 20
            root.add_child(LeaveSpaceInFront(total_dist))
            root.add_child(ChangeRoadBehavior(extra_space=self._extra_space))

        main_behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        main_behavior.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))

        # End condition
        end_condition = py_trees.composites.Sequence(name="End Condition")
        end_condition.add_child(WaitUntilInFront(self.ego_vehicles[0], self.other_actors[-1], check_distance=False))
        end_condition.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        main_behavior.add_child(end_condition)

        # Bicycle movement. Move them for a set distance, then stop
        offset = self._offset * self._starting_wp.lane_width / 2
        opt_dict = {'offset': offset}
        for actor, target_loc in zip(self.other_actors, self._target_locs):
            bicycle = py_trees.composites.Sequence(name="Bicycle behavior")
            bicycle.add_child(BasicAgentBehavior(actor, target_loc, target_speed=self._bicycle_speed, opt_dict=opt_dict))
            bicycle.add_child(HandBrakeVehicle(actor, 1))  # In case of collisions
            bicycle.add_child(WaitForever())  # Don't make the bicycle stop the parallel behavior
            main_behavior.add_child(bicycle)

        behavior = py_trees.composites.Sequence(name="Side lane behavior")
        behavior.add_child(InTriggerDistanceToVehicle(
            self.ego_vehicles[0], self.other_actors[0], self._trigger_distance))
        behavior.add_child(Idle(self._wait_duration))
        if self.route_mode:
            behavior.add_child(SwitchWrongDirectionTest(False))
            behavior.add_child(ChangeOppositeBehavior(spawn_dist=self._opposite_frequency))
        behavior.add_child(WaitForever())

        main_behavior.add_child(behavior)

        root.add_child(main_behavior)
        if self.route_mode:
            root.add_child(SwitchWrongDirectionTest(False))
            root.add_child(ChangeOppositeBehavior(spawn_dist=40))
            root.add_child(ChangeRoadBehavior(extra_space=0))

        for actor in self.other_actors:
            root.add_child(ActorDestroy(actor))

        return root
```