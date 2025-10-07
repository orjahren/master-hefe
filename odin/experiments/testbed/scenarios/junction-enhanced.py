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
import random # Added for randomized blueprints

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      SyncArrival,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower) # Added WaypointFollower for pedestrian
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

    # original crossing vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15

    # new parameters for added complexity
    _second_actor_target_velocity = 10 # Speed for the new crossing vehicle
    _pedestrian_speed = 1.0 # Pedestrian walk speed (m/s)
    _parked_car_count = 2 # Number of parked cars to add

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=90): # Increased timeout for a more complex scenario
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        self._second_actor_transform = None # Transform for the second crossing vehicle
        self._pedestrian_start_transform = None # Start transform for the pedestrian
        self._parked_car_transforms = [] # List to store transforms of parked cars

        # Timeout of scenario in seconds
        self.timeout = timeout

        super(NoSignalJunctionCrossing, self).__init__("NoSignalJunctionCrossing",
                                                       ego_vehicles,
                                                       config,
                                                       world,
                                                       debug_mode,
                                                       criteria_enable=criteria_enable)

        # Set adverse weather conditions and time of day to decrease driveability
        self.world.set_weather(carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=70.0,
            precipitation_deposits=50.0,
            wind_intensity=30.0,
            fog_density=30.0, # Increased fog
            wetness=70.0, # Increased wetness
            sun_azimuth_angle=270.0, # Setting sun to west (late evening)
            sun_altitude_angle=10.0 # Low sun angle / dusk conditions
        ))


    def _initialize_actors(self, config):
        """
        Custom initialization of actors for the enhanced scenario.
        """
        # Original other vehicle setup (crossing from left to right relative to ego)
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500), # Spawn far below to teleport
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        # --- Adding a second vehicle for increased complexity ---
        # This vehicle will approach from the opposite direction of the ego vehicle's path,
        # and will turn at the junction, creating a more complex intersection negotiation.
        # Assuming Town03 junction: Ego from (-75, -60) -> (-75, -170).
        # Original other vehicle: (-120, -136) -> (-30, -136).
        # Second vehicle comes from south on ego's road, turns right (west).
        if len(config.other_actors) > 1:
            self._second_actor_transform = config.other_actors[1].transform
        else: # Default transform if not provided by config (coming from opposite side of ego)
            self._second_actor_transform = carla.Transform(
                carla.Location(x=-74.63, y=-180, z=0.5), # Start further down ego's road, South
                carla.Rotation(pitch=0, yaw=90, roll=0)) # Heading North towards the junction

        second_vehicle_initial_transform = carla.Transform(
            carla.Location(self._second_actor_transform.location.x,
                           self._second_actor_transform.location.y,
                           self._second_actor_transform.location.z - 500), # Spawn far below
            self._second_actor_transform.rotation)
        second_vehicle_blueprint = CarlaDataProvider.get_blueprint_library().find('vehicle.audi.tt') # Default blueprint
        if randomize:
            second_vehicle_blueprint = random.choice(CarlaDataProvider.get_blueprint_library().filter('vehicle.*'))
        second_vehicle = CarlaDataProvider.request_new_actor(second_vehicle_blueprint.id, second_vehicle_initial_transform)
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)


        # --- Adding a pedestrian crossing the road after the junction ---
        # The pedestrian will cross the road segment where the original other actor passes.
        # This adds an additional hazard after the main intersection conflict.
        # Crossing at x ~ -45, y ~ -136 (near where the original crossing vehicle would proceed)
        self._pedestrian_start_transform = carla.Transform(
            carla.Location(x=-45, y=-130, z=0.5), # Start on one side of the road
            carla.Rotation(yaw=90)) # Facing to cross

        pedestrian_blueprint = CarlaDataProvider.get_blueprint_library().filter('walker.pedestrian.*')[0]
        if randomize:
            pedestrian_blueprint = random.choice(CarlaDataProvider.get_blueprint_library().filter('walker.pedestrian.*'))

        pedestrian = CarlaDataProvider.request_new_actor(pedestrian_blueprint.id, self._pedestrian_start_transform)
        pedestrian.set_simulate_physics(enabled=True)
        self.other_actors.append(pedestrian)


        # --- Adding parked cars to obscure view and narrow the path ---
        # These cars will be placed along the ego vehicle's approach to the junction.
        parked_car_blueprints = CarlaDataProvider.get_blueprint_library().filter('vehicle.*')
        for i in range(self._parked_car_count):
            parked_car_bp = random.choice(parked_car_blueprints) if randomize else CarlaDataProvider.get_blueprint_library().find('vehicle.chevrolet.impala')
            # Place cars along the left side of ego's lane, heading south
            parked_transform = carla.Transform(
                carla.Location(x=-79.0, y=-95.0 - (i * 10.0), z=0.5), # Offset from ego's lane
                carla.Rotation(yaw=180)) # Parked facing South
            parked_car = CarlaDataProvider.request_new_actor(parked_car_bp.id, parked_transform)
            parked_car.set_simulate_physics(enabled=False) # Static obstacle, does not move
            self.other_actors.append(parked_car) # Add to other_actors for cleanup
            self._parked_car_transforms.append(parked_transform) # Store transforms for completeness


    def _create_behavior(self):
        """
        Defines the behavior tree for the enhanced scenario, coordinating all actors.
        """

        # Retrieve actors created in _initialize_actors
        first_other_vehicle = self.other_actors[0]
        second_other_vehicle = self.other_actors[1]
        pedestrian = self.other_actors[2]
        # Parked cars are self.other_actors[3:] and are static, so they don't need behavior tree nodes.

        # Behavior for the ORIGINAL crossing vehicle
        original_vehicle_behavior = py_trees.composites.Sequence("OriginalVehicleBehavior")
        original_vehicle_behavior.add_child(ActorTransformSetter(first_other_vehicle, self._other_actor_transform))
        original_vehicle_behavior.add_child(InTriggerRegion(
            self.ego_vehicles[0], -80, -70, -75, -60, name="OriginalVehicle_EgoNearApproach")) # Ego vehicle approaches
        sync_original_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="SyncOriginalVehicleArrival")
        sync_original_parallel.add_child(SyncArrival(
            first_other_vehicle, self.ego_vehicles[0], carla.Location(x=-74.63, y=-136.34))) # Intersection point
        sync_original_parallel.add_child(InTriggerRegion(
            self.ego_vehicles[0], -90, -70, -124, -119, name="OriginalVehicle_EgoPassesTrigger")) # Ego passes a point
        original_vehicle_behavior.add_child(sync_original_parallel)
        keep_velocity_original_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OriginalVehicleKeepVelocityParallel")
        keep_velocity_original_parallel.add_child(KeepVelocity(first_other_vehicle, self._other_actor_target_velocity))
        keep_velocity_original_parallel.add_child(InTriggerRegion(
            first_other_vehicle, -45, -35, -140, -130, name="OriginalVehicleStopRegion"))
        original_vehicle_behavior.add_child(keep_velocity_original_parallel)
        original_vehicle_behavior.add_child(StopVehicle(first_other_vehicle, self._other_actor_max_brake))
        original_vehicle_behavior.add_child(ActorDestroy(first_other_vehicle))

        # Behavior for the SECOND crossing vehicle
        # This vehicle starts driving when ego is closer to the junction and proceeds to turn right (west)
        second_vehicle_behavior = py_trees.composites.Sequence("SecondVehicleBehavior")
        second_vehicle_behavior.add_child(ActorTransformSetter(second_other_vehicle, self._second_actor_transform))
        second_vehicle_behavior.add_child(InTriggerRegion(
            self.ego_vehicles[0], -90, -70, -150, -140, name="SecondVehicle_EgoNearJunction")) # Trigger later than original
        
        second_vehicle_junction_point = carla.Location(x=-74.63, y=-136.34) # Main junction point
        
        sync_second_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="SyncSecondVehicleArrival")
        # Sync the second vehicle's arrival at the junction with the ego vehicle, or when ego is past the first conflict.
        sync_second_parallel.add_child(SyncArrival(
            second_other_vehicle, self.ego_vehicles[0], second_vehicle_junction_point,
            trigger_point_min_distance=20, timeout=10)) # Activate when ego is within 20m of the junction point
        sync_second_parallel.add_child(InTriggerRegion(
            self.ego_vehicles[0], -80, -70, -130, -120, name="SecondVehicle_EgoMidJunction")) # Another ego trigger
        second_vehicle_behavior.add_child(sync_second_parallel)
        
        keep_velocity_second_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="SecondVehicleKeepVelocityParallel")
        keep_velocity_second_parallel.add_child(KeepVelocity(second_other_vehicle, self._second_actor_target_velocity))
        keep_velocity_second_parallel.add_child(InTriggerRegion(
            second_other_vehicle, -120, -110, -140, -130, name="SecondVehicleStopRegion")) # After turning right
        second_vehicle_behavior.add_child(keep_velocity_second_parallel)
        second_vehicle_behavior.add_child(StopVehicle(second_other_vehicle, self._other_actor_max_brake))
        second_vehicle_behavior.add_child(ActorDestroy(second_other_vehicle))

        # Behavior for the PEDESTRIAN
        pedestrian_behavior = py_trees.composites.Sequence("PedestrianBehavior")
        pedestrian_behavior.add_child(ActorTransformSetter(pedestrian, self._pedestrian_start_transform))
        pedestrian_behavior.add_child(InTriggerRegion(
            self.ego_vehicles[0], -60, -50, -145, -135, name="Pedestrian_EgoNearCrossing")) # Ego vehicle is near pedestrian crossing
        pedestrian_waypoint_list = [
            self._pedestrian_start_transform.location,
            carla.Location(x=-45, y=-140, z=0.5) # Walk to the other side of the road
        ]
        pedestrian_behavior.add_child(WaypointFollower(pedestrian, pedestrian_waypoint_list, self._pedestrian_speed, False))
        pedestrian_behavior.add_child(ActorDestroy(pedestrian))

        # Combine all dynamic actor behaviors into a parallel composite.
        # This ensures all complex interactions can happen simultaneously.
        dynamic_actors_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="DynamicActorsParallel")
        dynamic_actors_parallel.add_child(original_vehicle_behavior)
        dynamic_actors_parallel.add_child(second_vehicle_behavior)
        dynamic_actors_parallel.add_child(pedestrian_behavior)


        # Main scenario sequence
        root = py_trees.composites.Sequence("Root")
        scenario_sequence = py_trees.composites.Sequence("ScenarioSequence")
        
        # All dynamic actor behaviors run together after their individual triggers
        scenario_sequence.add_child(dynamic_actors_parallel)
        
        # End condition for the entire scenario
        scenario_sequence.add_child(InTriggerRegion(
            self.ego_vehicles[0], -90, -70, -170, -156, name="EgoReachedEndPoint"))

        root.add_child(scenario_sequence)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        # A single CollisionTest for the ego vehicle will check for collisions with any other actor,
        # including the new vehicles and the pedestrian.
        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

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