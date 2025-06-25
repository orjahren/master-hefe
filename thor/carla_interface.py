import carla


def get_carla_is_up() -> bool:
    """
    Check if the CARLA simulator is up and running.
    This function attempts to connect to the CARLA server and returns True if successful, otherwise False.
    """
    try:
        client = carla.Client('localhost', 2000)
        client.get_world()
        return True
    except Exception as e:
        print(f"CARLA connection failed: {e}")
        return False
