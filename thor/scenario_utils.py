import os
import subprocess
import time

debug = False


def get_scenario_list(scenario_repository_path: str) -> list:
    scenario_files = [
        f for f in os.listdir(scenario_repository_path)
        if f.endswith(".py")
    ]
    return scenario_files


def print_scenario_repo_stats(scenario_kind: str, scenario_repository_path: str) -> None:
    print(
        f"Scanning scenario repository at {scenario_repository_path} for {scenario_kind} scenarios...")
    scenarios = get_scenario_list(scenario_repository_path)
    print(f"Found {len(scenarios)} scenarios in {scenario_repository_path}")
    for scenario in scenarios:
        print(f"\t- {scenario}")


# TODO: There is proably a better way of doing this. Ideally we would have
# tracked this from the start, as part of enhanced scnario metadata.
def map_enhanced_filename_to_base(enhanced_filename: str, base_file_names: list[str]) -> str:

    manual_overrides = {
        "cut-in-mine.py": "cut_in.py",
        "enhanced_cut_in.py": "cut_in.py",
        "junction": "no_signal_junction_crossing.py",
        "cut_in": "cut_in.py",
        "follow": "follow_leading_vehicle.py",
        "route_obstacles": "route_obstacles.py",

    }
    for key in manual_overrides:
        if debug:
            print(f"Manual override check: {key} -> {manual_overrides[key]}")
        if key in enhanced_filename:
            if debug:
                print(
                    f"Using manual override mapping for {enhanced_filename} to {manual_overrides[key]}")
            return manual_overrides[key]

    stem = enhanced_filename.split(
        "-")[0]  # enhanced_filename without any -enhanced suffixes
    # TODO: Is this fine?
    for base_filename in base_file_names:
        if debug:
            print(
                f"Checking base filename: {base_filename} against stem: {stem}")
        if base_filename in stem or base_filename.split("_")[0] in stem:
            return base_filename
    raise ValueError(
        f"Could not map enhanced filename {enhanced_filename} to any base scenario.")


def pid_is_running(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True


def kill_pid(pid: int) -> bool:
    try:
        os.kill(pid, 15)  # Send SIGTERM
    except OSError:
        return False
    else:
        return True

# Returns true if execution probably was successful


def execute_scenario_by_scenario_runner(carla_pid: int, scenario_runner_root: str, enhanced_scenario_path: str, base_scenario_filename: str, scenario_name: str, log_folder: str) -> bool:

    # Copy enhanced scenario to scenario runner scenarios folder
    subprocess.run(
        f"cp {enhanced_scenario_path} {scenario_runner_root}/srunner/scenarios/{base_scenario_filename}",
        shell=True,
        cwd=scenario_runner_root
    )

    # Start scenario runner
    command = f"python3.7 {scenario_runner_root}/scenario_runner.py --scenario {scenario_name} --reloadWorld --record logs >> {log_folder}/scenario_runner.log 2>&1"
    scenario_proc = subprocess.Popen(
        command, shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd=scenario_runner_root  # Set working directory
    )

    ego_command = f"python3.10 {scenario_runner_root}/manual_control.py -a >> {log_folder}/ego_agent.log 2>&1"
    ego_proc = subprocess.Popen(
        ego_command, shell=True, cwd=scenario_runner_root)

    # Periodically check if scenario runner and Carla are alive
    try:
        while True:
            # If Carla has crashed/exited, poll() will not be None
            if scenario_proc.poll() is not None:
                print("Carla scenario runner is no longer live; exiting.")
                # ego_proc.terminate()
                ego_proc.kill()
                # Wait for both subprocesses to finish
                scenario_proc.wait()
                ego_proc.wait()
                return False

            if not pid_is_running(carla_pid):
                print("Carla simulator is no longer live; exiting.")
                scenario_proc.terminate()
                ego_proc.kill()
                # Wait for both subprocesses to finish
                scenario_proc.wait()
                ego_proc.wait()
                return False

            # Check every second
            time.sleep(1)

    except Exception as e:
        print(f"Exception occurred: {e}")
        ego_proc.terminate()
        scenario_proc.terminate()
        kill_pid(carla_pid)
        # Wait for both to finish
        scenario_proc.wait()
        ego_proc.wait()
        return False


# TODO: Is thre a better way to do this?
def scenario_filename_to_name(scenario_filename: str) -> str:
    # NOTE: Not all scenarios are used in this project as of writing.
    name_map = {
        "actor_flow.py": "ActorFlow",
        "background_activity.py": "BackgroundActivity",
        "background_activity_parametrizer.py": "BackgroundActivityParametrizer",
        "basic_scenario.py": "BasicScenario",
        "blocked_intersection.py": "BlockedIntersection",
        "change_lane.py": "ChangeLane",
        "change_lane_1.py": "ChangeLane_1",
        "construction_crash_vehicle.py": "ConstructionObstacle",
        "construction_1.py": "Construction_1",
        "control_loss.py": "ControlLoss",
        "cross_bicycle_flow.py": "CrossBicycleFlow",
        "cut_in.py": "CutInFrom_mine",
        "cut_in_with_static_vehicle.py": "CutInWithStaticVehicle",
        "freeride.py": "Freeride",
        "green_traffic_light.py": "GreenTrafficLight",
        "hard_break.py": "HardBreak",
        "highway_cut_in.py": "HighwayCutin",
        "invading_turn.py": "InvadingTurn",
        "maneuver_opposite_direction.py": "ManeuverOppositeDirection",
        "no_signal_junction_crossing.py": "NoSignalJunctionCrossing",
        "no_signal_junction_crossing_mod.py": "NoSignalJunctionCrossingMod",
        "object_crash_intersection.py": "ObjectCrashIntersection",
        "object_crash_vehicle.py": "ObjectCrashVehicle",
        "open_scenario.py": "OpenScenario",
        "opposite_vehicle_taking_priority.py": "OppositeVehicleTakingPriority",
        "osc2_scenario.py": "OSC2Scenario",
        "other_leading_vehicle.py": "OtherLeadingVehicle",
        "parking_cut_in.py": "ParkingCutIn",
        "parking_exit.py": "ParkingExit",
        "pedestrian_crossing.py": "PedestrianCrossing",
        "route_scenario.py": "RouteScenario",
        "signalized_junction_left_turn.py": "SignalizedJunctionLeftTurn",
        "signalized_junction_right_turn.py": "SignalizedJunctionRightTurn",
        "vehicle_opens_door.py": "VehicleOpensDoor",
        "yield_to_emergency_vehicle.py": "YieldToEmergencyVehicle",
        "cut-in-mine.py": "CutIn_mine",
        "cut-infrom-mine.py": "CutInFrom_mine",
        "cutinmine.py": "CutInMine",
        "route_obstacles.py": "Accident_1",
        "follow_leading_vehicle.py": "FollowLeadingVehicle_1",
        "slalom.py": "Slalom",
        "slalom_1.py": "Slalom_1",
        "vehicle_turning_right_8.py": "VehicleTurningRight_8",
        "priority_at_junction.py": "PriorityAtJunction",
        "vehicle_turning_right.py": "VehicleTurningRight",
        "vehicle_turning_right_1.py": "VehicleTurningRight_1",
        "lane_change_simple.py": "LaneChangeSimple",
    }
    res = name_map.get(scenario_filename, None)
    if not res:
        raise ValueError(
            f"Could not map scenario filename {scenario_filename} to scenario NAME.")
    return res


def remove_first_and_last_lines(input_path: str, output_path: str) -> str:
    with open(input_path, 'r') as file:
        lines = file.readlines()

    # Remove first and last lines (assumed to contain markdown ticks)
    if len(lines) > 2:
        lines = lines[1:-2]  # -2 to account for newline at end of file
    else:
        lines = []

    with open(output_path, 'w') as file:
        file.writelines(lines)
    return output_path
