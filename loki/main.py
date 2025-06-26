

import requests

# from odin.server import ODIN_PORT
# from thor.server import THOR_PORT

ODIN_PORT = 4000
THOR_PORT = 5000


# TODO: Merge health checks into a single function?
def do_thor_health_check():
    try:
        res = requests.get(f"http://localhost:{THOR_PORT}/health")
    except requests.ConnectionError:
        print("Thor server is not running or unreachable.")
        exit(1)
    if res.status_code == 200:
        print("Thor is healthy.")
    else:
        print("Thor health check failed.")
        exit(1)


def do_odin_health_check():
    try:
        res = requests.get(f"http://localhost:{ODIN_PORT}/health")
    except requests.ConnectionError:
        print("Odin server is not running or unreachable.")
        exit(1)
    if res.status_code == 200:
        print("Odin is healthy.")
    else:
        print("Odin health check failed.")
        exit(1)


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


def get_enhanced_test_case(test_case_id):
    print(f"Enhancing test case: {test_case_id}")

    # Enhance test case on Odin
    result = requests.post(
        f"http://localhost:{ODIN_PORT}/enhance_test_case",
        json={"test_case_id": test_case_id})
    if result.status_code != 200:
        print(f"Failed to enhance test case: {test_case_id}")
        return None
    print(f"Test case {test_case_id} enhanced successfully.")
    value = result.json().get("result", "No result found")
    return value


def get_improvement(base_test_case, enhanced_test_case):
    # Simulate getting an improvement between two test cases
    # TODO: Implement actual logic to compare test cases
    return f"Improvement from {base_test_case} to {enhanced_test_case}"


if __name__ == "__main__":
    print("Loki is running...")

    do_thor_health_check()
    do_odin_health_check()

    test_case_id = "test_case_123"

    print("Starting test case execution...")
    base_result = run_test_case(test_case_id)
    print(f"Base result: {base_result}")

    enhanced_test_case = get_enhanced_test_case(test_case_id)

    enhanced_test_case_result = run_test_case(enhanced_test_case)
    print(
        f"Enhanced test case execution completed with result: {enhanced_test_case_result}")

    improvement = get_improvement(base_result, enhanced_test_case_result)
    print(f"Improvement: {improvement}")

    print("Loki execution finished.")
