# RUN test cases on simulator

from fastapi import Request
from contextlib import asynccontextmanager
import pika
import threading
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, PlainTextResponse

from carla_interface import get_carla_is_up, set_map, test_spawn_some_vehicles

THOR_PORT = 6000


def rabbitmq_listener():
    print("Starting RabbitMQ listener...")

    def callback(ch, method, properties, body):
        print(f"Received message from RabbitMQ: {body.decode()}")

    credentials = pika.PlainCredentials('user', 'pass')
    print(
        f"Connecting to RabbitMQ with credentials...")
    connection = pika.BlockingConnection(
        pika.ConnectionParameters('rabbit', credentials=credentials)
    )
    print("Connected to RabbitMQ server.")
    channel = connection.channel()
    print("Channel declared.")
    channel.queue_declare(queue='test_queue')
    print("Queue 'test_queue' declared.")
    channel.basic_consume(queue='test_queue',
                          on_message_callback=callback, auto_ack=True)
    print("RabbitMQ consumer set up. Waiting for messages...")
    channel.start_consuming()


@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Lifespan: Starting Thor server...")
    listener_thread = threading.Thread(target=rabbitmq_listener, daemon=True)
    listener_thread.start()
    yield
    print("Lifespan: Shutting down Thor server...")


app = FastAPI(lifespan=lifespan)


@app.get("/", response_class=PlainTextResponse)
async def read_root() -> str:
    return "Hello from Thor!"


@app.get('/health')
async def health_check():
    return {"status": "healthy" if get_carla_is_up() else "unhealthy"}


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
    print("Hitting endpoint for checking if CARLA is up...")
    res = get_carla_is_up()
    if res:
        print("CARLA is up and running.")
        return JSONResponse(content={"carla_is_up": True})
    else:
        print("CARLA is not reachable.")
        return JSONResponse(content={"carla_is_up": False}, status_code=503)


@app.get("/test_carla")
async def test_carla() -> JSONResponse:
    print("Hitting endpoint for testing CARLA functionality...")
    result = test_spawn_some_vehicles()
    return JSONResponse(content={"test_spawn_some_vehicles": result})


@app.post("/test_rabbit")
async def test_rabbit(request: Request) -> JSONResponse:
    data = await request.json()
    message = data.get("message", "")
    print(
        f"Hitting endpoint for RabbitMQ tester... Received message: {message}")
    return JSONResponse(content={"status": "RabbitMQ test endpoint is up", "received_message": message})


@app.post("/set_map")
async def set_map_endpoint(request: Request) -> JSONResponse:
    data = await request.json()
    map_name = data.get("map_name", "Town01")
    print(f"Setting CARLA map to: {map_name}")
    try:
        world = set_map(map_name)
        return JSONResponse(content={"status": "success", "map_name": map_name})
    except Exception as e:
        print(f"Error setting CARLA map: {e}")
        return JSONResponse(content={"status": "error", "message": str(e)}, status_code=500)
