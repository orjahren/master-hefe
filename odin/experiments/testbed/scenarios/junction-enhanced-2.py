```python
#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Non-signalized junctions: crossing negotiation:

The hero vehicle is passing through a junction without traffic lights
And encounters another vehicle passing across the junction.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      SyncArrival,
                                                                      KeepVelocity,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerRegion, DriveDistance, WaitEndIntersection
from srunner.scenarios.basic_scenario import BasicScenario


class NoSignalJunctionCrossing(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15

    # New parameters for added complexity
    _second_actor_target_velocity = 12
    _parked_car_model = 'vehicle.tesla.model3' # Using a common model for the static obstacle

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        self._second_actor_transform = None # Added for the second crossing vehicle
        self._parked_car_transform = None # Added for the static obstacle
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(NoSignalJunctionCrossing, self).__init__("NoSignalJunctionCrossing",
                                                       ego_vehicles,
                                                       config,
                                                       world,
                                                       debug_mode,
                                                       criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Original other actor
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500), # Spawn far below ground initially
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        # Second other actor (new for complexity)
        second_actor_model = None
        if len(config.other_actors) > 1:
            self._second_actor_transform = config.other_actors[1].transform # Use transform from config if available
            second_actor_model = config.other_actors[1].model
        else:
            # Fallback: Define a default transform and model if only one actor is provided in config
            self._second_actor_transform = carla.Transform(carla.Location(x=-75.0, y=-150.0, z=0.5), carla.Rotation(yaw=90)) # South approach, facing North
            second_actor_model = 'vehicle.bh.crossbike' # Default model if no second actor in config

        second_vehicle_transform = carla.Transform(
            carla.Location(self._second_actor_transform.location.x,
                           self._second_actor_transform.location.y,
                           self._second_actor_transform.location.z - 500), # Spawn far below ground initially
            self._second_actor_transform.rotation)
        second_vehicle = CarlaDataProvider.request_new_actor(second_actor_model, second_vehicle_transform)
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)


        # Parked car (static obstacle - new for complexity)
        # Placed on ego's right side, slightly obstructing view or narrowing the lane before the junction
        self._parked_car_transform = carla.Transform(carla.Location(x=-85.0, y=-115.0, z=0.5), carla.Rotation(yaw=-90.0))
        parked_car = CarlaDataProvider.request_new_actor(self._parked_car_model, self._parked_car_transform)
        parked_car.set_simulate_physics(enabled=False) # Static obstacle, no physics simulation
        self.other_actors.append(parked_car) # Add to other_actors for proper cleanup

        # Set adverse weather conditions (new for complexity)
        # Decreases visibility and road grip, making the scenario more challenging
        self.world.set_weather(carla.WeatherParameters(cloudiness=80.0, precipitation=80.0, fog_density=30.0, sun_altitude_angle=-20.0))


    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes for original other actor
        start_other_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -80, -70,
            -75, -60)

        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            carla.Location(x=-74.63, y=-136.34))

        pass_through_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -90, -70,
            -124, -119)

        keep_velocity_other = KeepVelocity(
            self.other_actors[0],
            self._other_actor_target_velocity)

        stop_other_trigger = InTriggerRegion(
            self.other_actors[0],
            -45, -35,
            -140, -130)

        stop_other = StopVehicle(
            self.other_actors[0],
            self._other_actor_max_brake)

        end_condition = InTriggerRegion(
            self.ego_vehicles[0],
            -90, -70,
            -170, -156
        )

        # New leaf nodes for the second other actor (added for complexity)
        # This actor will approach from a different direction, potentially colliding with either ego or the first other actor
        sync_arrival_second = SyncArrival(
            self.other_actors[1], self.ego_vehicles[0],
            carla.Location(x=-74.63, y=-119.0)) # Sync point adjusted for second actor's path

        keep_velocity_second = KeepVelocity(
            self.other_actors[1],
            self._second_actor_target_velocity)

        stop_second_trigger = InTriggerRegion(
            self.other_actors[1],
            -85, -65, # Region where the second actor should stop after passing
            -100, -80)

        stop_second = StopVehicle(
            self.other_actors[1],
            self._other_actor_max_brake) # Using same brake force for simplicity


        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        # Original parallel compositions
        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # New parallel composition for the second actor (added for complexity)
        sync_arrival_second_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_second_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        # Original actor setup
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        # Second actor setup (new)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self._second_actor_transform))
        # The parked car's transform is set in _initialize_actors as it's static.

        scenario_sequence.add_child(start_other_trigger) # This trigger starts the scenario for all relevant actors

        # Run both sync behaviors in parallel (added for complexity)
        both_sync_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        both_sync_parallel.add_child(sync_arrival_parallel)
        both_sync_parallel.add_child(sync_arrival_second_parallel) # Add second actor's sync behavior
        scenario_sequence.add_child(both_sync_parallel)

        # Run both keep velocity behaviors in parallel (added for complexity)
        both_keep_velocity_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        both_keep_velocity_parallel.add_child(keep_velocity_other_parallel)
        both_keep_velocity_parallel.add_child(keep_velocity_second_parallel) # Add second actor's keep velocity behavior
        scenario_sequence.add_child(both_keep_velocity_parallel)

        # Add stop behaviors for both dynamic actors in parallel
        stop_all_parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        stop_all_parallel.add_child(stop_other)
        stop_all_parallel.add_child(stop_second)
        scenario_sequence.add_child(stop_all_parallel)

        scenario_sequence.add_child(end_condition)
        # Destroy all dynamic actors, including the newly added parked car
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[1]))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[2])) # For the parked car

        # Building original sync_arrival_parallel
        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(pass_through_trigger) # This trigger is for ego approaching junction

        # Building original keep_velocity_other_parallel
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

        # Building new sync_arrival_second_parallel (added for complexity)
        sync_arrival_second_parallel.add_child(sync_arrival_second)
        sync_arrival_second_parallel.add_child(pass_through_trigger) # Reusing ego's trigger for simplicity

        # Building new keep_velocity_second_parallel (added for complexity)
        keep_velocity_second_parallel.add_child(keep_velocity_second)
        keep_velocity_second_parallel.add_child(stop_second_trigger)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

        # The base scenario only checks ego vehicle collision.
        # Adding criteria for collision with new actors is implicit by CollisionTest(self.ego_vehicles[0])
        # as it checks collision with ANY other actor.
        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class NoSignalJunctionCrossingRoute(BasicScenario):

    """
    At routes, these scenarios are simplified, as they can be triggered making
    use of the background activity. For unsignalized intersections, just wait
    until the ego_vehicle has left the intersection.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        # Timeout of scenario in seconds
        self.timeout = timeout
        self._end_distance = 50

        super(NoSignalJunctionCrossingRoute, self).__init__("NoSignalJunctionCrossingRoute",
                                                            ego_vehicles,
                                                            config,
                                                            world,
                                                            debug_mode,
                                                            criteria_enable=criteria_enable)

    def _create_behavior(self):
        """
        Just wait for the ego to exit the junction, for route the BackgroundActivity already does all the job
        """
        sequence = py_trees.composites.Sequence("UnSignalizedJunctionCrossingRoute")
        sequence.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._end_distance))
        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        return []

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
```