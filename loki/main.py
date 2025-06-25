
import random


def run_test_case(test_case_id):
    print(f"Running test case: {test_case_id}")
    # Simulate running a test case
    result = f"Result of {test_case_id}: {random.randint(1, 100)}"
    return result


if __name__ == "__main__":
    print("Loki is running...")

    test_case_id = "test_case_123"

    print("Starting test case execution...")
    result = run_test_case(test_case_id)
    print(f"Test case execution completed with result: {result}")
    print("Loki execution finished.")
