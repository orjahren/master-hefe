import os
import subprocess
import sys
import time

from thor.carla_interface import get_carla_is_up
from thor.scenario_utils import get_scenario_list, map_enhanced_filename_to_base, remove_first_and_last_lines, scenario_filename_to_name, execute_scenario_by_scenario_runner

# We start Carla and run various scenarios

# We expect that Carla will crash. When this happens, we must record the
# failure and restart it and continue with the next scenario.


def get_env_var(var_name):
    value = os.environ.get(var_name, "")
    if value == "":
        print(f"Env var {var_name} not set")
        sys.exit(1)
    return value


# You won't get far without these...
assert get_env_var("CARLA_ROOT"), "CARLA_ROOT not set"
assert get_env_var("SCENARIO_RUNNER_ROOT"), "SCENARIO_RUNNER_ROOT not set"
assert get_env_var("HEFE_ROOT"), "HEFE_ROOT not set"

# Change to "prod" for production runs. Keep records separate to assert their validity.
MODE = "dev"

# Note how we assert this to exist above
LOG_FOLDER = get_env_var("HEFE_ROOT") + "/thor/scenario-benchmarker/" + MODE


# Start Carla, wait until its up and return its PID
def start_carla() -> int:
    carla_root = get_env_var("CARLA_ROOT")
    print("Starting Carla...")

    # Start Carla in the background and get its PID
    carla_cmd = f"nohup {carla_root}/CarlaUE4.sh >> {LOG_FOLDER}/carla.log 2>&1"
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


def write_to_logfile(message: str) -> None:
    with open(LOG_FOLDER + "/experiments.log", "a") as f:
        f.write(message + "\n")


def test_name_mapping(base_scenarios: list[str], enhanced_scenarios: list[str], debug: bool) -> None:
    # Scenario file names:
    for enhanced_scenario in enhanced_scenarios:
        if debug:
            print(
                f"Testing mapping for enhanced scenario: {enhanced_scenario}")
        # Will throw if it fails
        mapped_base = map_enhanced_filename_to_base(
            enhanced_scenario, base_scenarios)
        if debug:
            print(
                f"\tEnhanced scenario {enhanced_scenario} maps to base scenario {mapped_base}")

    # Scenario content names needed for running:
    for base_scenario in base_scenarios:
        if debug:
            print(
                f"Testing mapping for base scenario: {base_scenario}")
        # Will throw if it fails
        output_name = scenario_filename_to_name(
            base_scenario)
        if debug:
            print(
                f"\tBase scenario {base_scenario} maps to output name {output_name}")
    return True


def get_current_time_formatted() -> str:
    return time.strftime("%H:%M:%S", time.gmtime(time.time()))


IGNORED_FILES = ["__init__.py", "common.py", "cut_in.py", "enhanced_cut_in.py"]


def scenario_benchmarker():
    # Placeholder for the main benchmarking logic
    print("Running scenario benchmarker...")
    write_to_logfile("Starting scenario benchmarker at time" +
                     get_current_time_formatted())
    # ... existing benchmarking code ...

    successful_runs = 0

    hefe_root = get_env_var("HEFE_ROOT")
    enhanced_scenario_path = f"{hefe_root}/odin/experiments/testbed/scenarios"
    enhanced_scenarios = list(filter(
        lambda x: x not in IGNORED_FILES and not "prepped" in x, get_scenario_list(enhanced_scenario_path)))
    # print_scenario_repo_stats("Enhanced", enhanced_scenario_path)

    scenario_runner_root = get_env_var("SCENARIO_RUNNER_ROOT")
    # print(f"Using scenario runner at {scenario_runner_root}")

    base_scenarios = list(filter(lambda x: x not in IGNORED_FILES, get_scenario_list(
        scenario_runner_root + "/srunner/scenarios")))
    # print_scenario_repo_stats(
    # "Base", scenario_runner_root + "/srunner/scenarios/")

    # Precompute prepped names to test them before running
    prepped_mappings = [
        map_enhanced_filename_to_base(
            remove_first_and_last_lines(
                enhanced_scenario_path + "/" + es,
                enhanced_scenario_path + "/" + es.replace(".py", "-prepped.py")
            ),
            base_scenarios
        )
        for es in enhanced_scenarios if es not in IGNORED_FILES and not "prepped" in es
    ]

    assert test_name_mapping(
        base_scenarios, enhanced_scenarios + prepped_mappings, False), "Not all scenarios mapped correctly."
    # exit(0)

    carla_pid = start_carla()
    for enhanced_scenario in enhanced_scenarios:
        if not pid_is_running(carla_pid):
            print("Carla has crashed! Restarting...")
            write_to_logfile("Carla crash detected at " +
                             get_current_time_formatted())
            carla_pid = start_carla()

        base_scenario = map_enhanced_filename_to_base(
            enhanced_scenario, base_scenarios)
        scenario_name = scenario_filename_to_name(base_scenario)

        # Prep by removing first and last lines containing md ticks
        prepped_scenario = remove_first_and_last_lines(
            enhanced_scenario_path + "/" + enhanced_scenario, enhanced_scenario_path + "/" + enhanced_scenario.replace(".py", "-prepped.py"))

        print(
            f"Running enhanced scenario {enhanced_scenario} mapped to prepped scenario {prepped_scenario} and base scenario {base_scenario}...")
        write_to_logfile(
            f"Running scenario {scenario_name} at {get_current_time_formatted()}")

        status = execute_scenario_by_scenario_runner(
            carla_pid,
            scenario_runner_root,
            prepped_scenario,
            base_scenario,
            scenario_name,
            LOG_FOLDER
        )
        successful_runs += int(status)
        write_to_logfile(
            f"Finished running scenario {enhanced_scenario} at {get_current_time_formatted()}\n\n\n")

    cleanup(carla_pid)

    # NOTE: Dette tallet stemmer ikke
    print(
        f"Scenario benchmarking complete. {successful_runs} out of {len(enhanced_scenarios)} scenarios ran successfully.")
    write_to_logfile(
        f"Scenario benchmarking complete at {get_current_time_formatted()}. {successful_runs} out of {len(enhanced_scenarios)} scenarios ran successfully.")


if __name__ == "__main__":
    scenario_benchmarker()
