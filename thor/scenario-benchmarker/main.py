import os
import subprocess
import sys
import time

from thor.carla_interface import get_carla_is_up
from thor.scenario_utils import get_scenario_list, map_enhanced_filename_to_base, print_scenario_repo_stats

# We start Carla and run various scenarios

# We expect that Carla will crash. When this happens, we must record the
# failure and restart it and continue with the next scenario.


def get_env_var(var_name):
    value = os.environ.get(var_name, "")
    if value == "":
        print(f"Env var {var_name} not set")
        sys.exit(1)
    return value


BENCHMARKER_LOGFILE = "scenario_benchmarker.log"


# Start Carla, wait until its up and return its PID
def start_carla() -> int:
    carla_root = get_env_var("CARLA_ROOT")
    print("Starting Carla...")

    # Start Carla in the background and get its PID
    carla_cmd = f"nohup {carla_root}/CarlaUE4.sh > {BENCHMARKER_LOGFILE} 2>&1"
    proc = subprocess.Popen(carla_cmd, shell=True)
    print(f"Carla started with PID {proc.pid}")

    print("Waiting for Carla to start...")
    for i in range(60):  # Wait up to 60 seconds
        if get_carla_is_up("localhost"):
            print("Carla started.")
            return proc.pid
        else:
            print("Carla not up yet, retrying...")
            time.sleep(1)
    print("Carla did not start within 60 seconds.")

    sys.exit(1)


def pid_is_running(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True


def cleanup(carla_pid: int) -> None:
    print("Cleaning up...")
    try:
        os.kill(carla_pid, 15)  # Send SIGTERM
        print("Carla terminated.")
    except OSError as e:
        print(f"Error terminating Carla: {e}")


def test_name_mapping(base_scenarios: list[str], enhanced_scenarios: list[str], debug: bool) -> None:
    for enhanced_scenario in enhanced_scenarios:
        if debug:
            print(
                f"Testing mapping for enhanced scenario: {enhanced_scenario}")
        # Will throw if it fails
        mapped_base = map_enhanced_filename_to_base(
            enhanced_scenario, base_scenarios)
        if debug:
            print(
                f"Enhanced scenario {enhanced_scenario} maps to base scenario {mapped_base}")
    return True


def scenario_benchmarker():
    # Placeholder for the main benchmarking logic
    print("Running scenario benchmarker...")
    # ... existing benchmarking code ...

    hefe_root = get_env_var("HEFE_ROOT")
    enhanced_scenario_path = f"{hefe_root}/odin/experiments/testbed/scenarios/"
    enhanced_scenarios = get_scenario_list(enhanced_scenario_path)
    # print_scenario_repo_stats("Enhanced", enhanced_scenario_path)

    scenario_runner_root = get_env_var("SCENARIO_RUNNER_ROOT")
    # print(f"Using scenario runner at {scenario_runner_root}")

    base_scenarios = get_scenario_list(
        scenario_runner_root + "/srunner/scenarios/")
    # print_scenario_repo_stats(
    # "Base", scenario_runner_root + "/srunner/scenarios/")

    assert test_name_mapping(
        base_scenarios, enhanced_scenarios, False), "Not all scenarios mapped correctly."
    exit(0)

    carla_pid = start_carla()
    while (True):
        if not pid_is_running(carla_pid):
            print("Carla has crashed! Restarting...")
            carla_pid = start_carla()
        # Run scenario benchmarking steps
        if (inp := input("Press Enter to run the next scenario...")) == "q":
            print("Exiting scenario benchmarker.")
            break
        print("You said", inp)

    cleanup(carla_pid)


if __name__ == "__main__":
    scenario_benchmarker()
