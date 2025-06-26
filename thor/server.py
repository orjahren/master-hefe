# RUN test cases on simulator

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, PlainTextResponse

from carla_interface import get_carla_is_up

THOR_PORT = 5000

app = FastAPI()


@app.get("/", response_class=PlainTextResponse)
async def read_root() -> str:
    return "Hello from Thor!"


@app.get('/health')
async def health_check():
    return {"status": "healthy"}


@app.post('/run_test_case')
async def run_test_case(request: Request):
    data = await request.json()
    test_case_id = data.get('test_case_id', 'default_test_case')
    print(f"Running test case: {test_case_id}")
    # Simulate test case execution
    result = f"Result for {test_case_id}"
    return {"result": result}


@app.get("/carla_is_up")
async def carla_is_up() -> JSONResponse:
    """
    Check if the CARLA simulator is up and running.
    This function attempts to connect to the CARLA server and returns True if successful, otherwise False.
    """
    print("Checking if CARLA is up...")
    res = get_carla_is_up()
    if res:
        print("CARLA is up and running.")
        return JSONResponse(content={"carla_is_up": True})
    else:
        print("CARLA is not reachable.")
        return JSONResponse(content={"carla_is_up": False}, status_code=503)
