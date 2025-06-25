# RUN test cases on simulator

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, PlainTextResponse

from thor.carla_interface import get_carla_is_up

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
    This function attempts t
    """
    return get_carla_is_up()
