

import requests

from thor.server import THOR_PORT


def do_thor_health_check():
    res = requests.get(f"http://localhost:{THOR_PORT}/health")
    if res.status_code == 200:
        print("Loki is healthy.")
    else:
        print("Loki health check failed.")


def run_test_case(test_case_id):
    print(f"Running test case: {test_case_id}")

    # Run test case on Loki
    result = requests.post(
        f"http://localhost:{THOR_PORT}/run_test_case",
        json={"test_case_id": test_case_id})
    if result.status_code != 200:
        print(f"Failed to run test case: {test_case_id}")
        return None
    print(f"Test case {test_case_id} executed successfully.")
    value = result.json().get("result", "No result found")
    return value


if __name__ == "__main__":
    print("Loki is running...")

    test_case_id = "test_case_123"

    print("Starting test case execution...")
    result = run_test_case(test_case_id)
    print(f"Test case execution completed with result: {result}")
    print("Loki execution finished.")
