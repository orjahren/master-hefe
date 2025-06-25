# Flask app for running the Thor server
import random
from flask import Flask, jsonify, request

THOR_PORT = 5000


app = Flask(__name__)


@app.route('/health', methods=['GET'])
def health_check():
    # Simulate a health check
    return jsonify({"status": "healthy"}), 200


def get_spoofed_result():
    # Simulate a spoofed result
    return random.randint(1, 100)


@app.route('/run_test_case', methods=['POST'])
def run_test_case():
    data = request.get_json()
    test_case_id = data.get('test_case_id', 'default_test_case')

    result = get_spoofed_result()

    print(f"Running test case: {test_case_id}: {result}")
    # Simulate running a test case
    result = f"Result of {test_case_id}: {result}"
    return jsonify({"result": result})


if __name__ == '__main__':
    app.run(port=THOR_PORT, debug=True)
    print(f"Thor server is running on port {THOR_PORT}")
    print("You can access the health check at /health and run test cases at /run_test_case")
    print(
        "Example: POST to /run_test_case with JSON body {'test_case_id': 'your_test_case_id'}")
    print("Press Ctrl+C to stop the server.")
    print("Thor server is ready to accept requests.")
