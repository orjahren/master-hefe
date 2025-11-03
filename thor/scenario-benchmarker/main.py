import os
import subprocess
import sys
import time
import socket

from thor.carla_interface import get_carla_is_up

# We start Carla and run various scenarios

# We expect that Carla will crash. When this happens, we must record the
# failure and restart it and continue with the next scenario.


def get_env_var(var_name):
    value = os.environ.get(var_name, "")
    if value == "":
        print(f"{var_name} not set")
        sys.exit(1)
    return value


BENCHMARKER_LOGFILE = "scenario_benchmarker.log"


def start_carla() -> int:
    carla_root = get_env_var("CARLA_ROOT")
    print("Starting Carla...")

    # Start Carla in the background and get its PID
    carla_cmd = f"nohup {carla_root}/CarlaUE4.sh > {BENCHMARKER_LOGFILE} 2>&1"
    proc = subprocess.Popen(carla_cmd, shell=True)
    print(f"Carla started with PID {proc.pid}")

    # Wait for Carla to be up (try to connect to localhost:2000)
    print("Waiting for Carla to start...")
    # TODO: Can refactor this
    for i in range(60):  # Wait up to 60 seconds
        # TODO: Should probably use carla package
        if get_carla_is_up("localhost"):
            print("Carla started.")
            break
        else:
            print("Carla not up yet, retrying...")
            time.sleep(1)
    else:
        print("Carla did not start within 60 seconds.")

        sys.exit(1)

    return proc.pid


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


def get_scenario_list(scenario_repository_path: str) -> list:
    scenario_files = [
        f for f in os.listdir(scenario_repository_path)
        if f.endswith(".py")
    ]
    return scenario_files


def scenario_benchmarker():
    # Placeholder for the main benchmarking logic
    print("Running scenario benchmarker...")
    # ... existing benchmarking code ...

    hefe_root = get_env_var("HEFE_ROOT")
    enhanced_scenario_path = f"{hefe_root}/odin/experiments/testbed/scenarios/"
    print(f"Looking for scenarios in {enhanced_scenario_path}")
    scenarios = get_scenario_list(enhanced_scenario_path)
    print(f"Found {len(scenarios)} scenarios to benchmark.")
    for scenario in scenarios:
        print(f"Scenario: {scenario}")
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
