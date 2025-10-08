import carla

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    print("Available pedestrian blueprints:")
    for bp in blueprint_library.filter('walker.pedestrian.*'):
        print(f"- {bp.id}")

if __name__ == '__main__':
    main()

