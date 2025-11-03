import os

debug = False


def get_scenario_list(scenario_repository_path: str) -> list:
    scenario_files = [
        f for f in os.listdir(scenario_repository_path)
        if f.endswith(".py")
    ]
    return scenario_files


def print_scenario_repo_stats(scenario_kind: str, scenario_repository_path: str) -> None:
    print(
        f"Scanning scenario repository at {scenario_repository_path} for {scenario_kind} scenarios...")
    scenarios = get_scenario_list(scenario_repository_path)
    print(f"Found {len(scenarios)} scenarios in {scenario_repository_path}")
    for scenario in scenarios:
        print(f"\t- {scenario}")


# TODO: There is proably a better way of doing this. Ideally we would have
# tracked this from the start, as part of enhanced scnario metadata.
def map_enhanced_filename_to_base(enhanced_filename: str, base_file_names: list[str]) -> str:
    stem = enhanced_filename.split(
        "-")[0]  # enhanced_filename without any -enhanced suffixes
    # TODO: Is this fine?
    for base_filename in base_file_names:
        if debug:
            print(
                f"Checking base filename: {base_filename} against stem: {stem}")
        if base_filename in stem or base_filename.split("_")[0] in stem:
            return base_filename
    raise ValueError(
        f"Could not map enhanced filename {enhanced_filename} to any base scenario.")
