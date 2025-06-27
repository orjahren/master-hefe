# RUN test cases on simulator

from contextlib import asynccontextmanager
import pika
import threading
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, PlainTextResponse
import traceback
import time

from carla_interface import get_carla_is_up, test_spawn_some_vehicles

THOR_PORT = 5000


# NOTE: Nå prøver den å koble til RabbitMQ før den podden er klar, som skaper
# litt støy i loggene, og må håndteres med retries.
# TODO: Burde fixe noe mer robust opplegg for å håndtere den casen.
def rabbitmq_listener():
    print("Starting RabbitMQ listener...")

    def callback(ch, method, properties, body):
        print(f"Received message from RabbitMQ: {body.decode()}")

    max_retries = 10
    for attempt in range(1, max_retries + 1):
        try:
            credentials = pika.PlainCredentials('user', 'pass')
            print(
                f"Connecting to RabbitMQ with credentials... (attempt {attempt})")
            connection = pika.BlockingConnection(
                pika.ConnectionParameters('localhost', credentials=credentials)
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
            break  # Exit loop if successful
        except Exception as e:
            print(f"RabbitMQ listener error (attempt {attempt}): {e}")
            traceback.print_exc()
            if attempt < max_retries:
                print("Retrying in 3 seconds...")
                time.sleep(3)
            else:
                print("Failed to connect to RabbitMQ after several attempts.")


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
async def test_rabbit() -> JSONResponse:
    print("Hitting endpoint for RabbitMQ tester...")
    return JSONResponse(content={"status": "RabbitMQ test endpoint is up"})
