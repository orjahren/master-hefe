import carla

CARLA_HOST = "localhost"
CARLA_PORT = 2000


def get_carla_is_up() -> bool:
    """
    Check if the CARLA simulator is up and running.
    This function attempts to connect to the CARLA server and returns True if successful, otherwise False.
    """
    try:
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.get_world()
        return True
    except Exception as e:
        print(f"CARLA connection failed: {e}")
        return False
