
# We wish to decrease the driveability of the scenario by enhancing it with more
# details, increasing its complexity

# Prompt structure:
"""
1 - Context: We are working with a driving simulation environment for the Carla simulator.
2 - Task: Decrease the driveability of the scenario by enhancing it with more details and complexity.
3 - Input: <scenario_description, in python carla scenario format>
4 - Output: An enhanced version of the scenario description with additional
details and complexity, still in Python carla scenario format. ONLY output the
code, without any additional text or explanation.
"""

PROMPTS = [
    lambda python_carla_scenario_raw: f"""
    1 - Context: We are working with a driving simulation environment for the Carla simulator.
    2 - Task: Decrease the driveability of the scenario by enhancing it with more details and complexity.
    3 - Input: {python_carla_scenario_raw}
    4 - Output: An enhanced version of the scenario description with additional
    details and complexity, still in Python carla scenario format.
    """,
    lambda python_carla_scenario_raw: f"""
    1 - Context: You are a tool for decreasing the driveability of scenarios in the driving simulator Carla.
    2 - Task: Decrease the driveability of the scenario by enhancing it with more details and complexity.
    3 - Input, the Python specification for the scenario: {python_carla_scenario_raw}
    4 - Output: An enhanced version of the scenario with additional details and
    complexity, still in Python carla scenario format. Only ever output the code,
    without any additional text or explanation.
    """,
    lambda python_carla_scenario_raw: f"""
    1 - Context: You are a tool for decreasing the driveability of scenarios in the driving simulator Carla.
    2 - Task: Decrease the driveability of the scenario by enhancing it with more details and complexity.
    3 - Input, the Python specification for the scenario: {python_carla_scenario_raw}
    4 - Output: An enhanced version of the scenario with additional details and
    complexity, still in Python carla scenario format. Only ever output the code,
    without any additional text or explanation. It is important that you only
    use methods and classes that are part of the official Carla API, and do not
    invent new ones or use non-existent ones.
    """,

    # Chain of thought prompting
    lambda python_carla_scenario_raw: f"""
    1 - Context: You are a tool for decreasing the driveability of scenarios in the driving simulator Carla.
    2 - Task: Decrease the driveability of the scenario by enhancing it with more details and complexity.
    3 - Input, the Python specification for the scenario: {python_carla_scenario_raw}
    4 - Reasoning: Think step by step about how to make the scenario more complex and less driveable, considering possible obstacles, traffic, weather, and other factors using only the official Carla API.
    5 - Output: Only output the enhanced scenario code in Python Carla scenario format, with no additional text or explanation.
    """,
    # Samme som den over bare med "kun bruk metoder som finnes i filen"
    lambda python_carla_scenario_raw: f"""
    1 - Context: You are a tool for decreasing the driveability of scenarios in the driving simulator Carla.
    2 - Task: Decrease the driveability of the scenario by enhancing it with
    more details and complexity, using only the same methods that are already
    provided in the file.
    3 - Input, the Python specification for the scenario: {python_carla_scenario_raw}
    4 - Reasoning: Think step by step about how to make the scenario more complex and less driveable, considering possible obstacles, traffic, weather, and other factors using only the official Carla API.
    5 - Output: Only output the enhanced scenario code in Python Carla scenario format, with no additional text or explanation.
    """,
    # Samme som den over bare med "kun bruk Carla-APIet"
    lambda python_carla_scenario_raw: f"""
    1 - Context: You are a tool for decreasing the driveability of scenarios in the driving simulator Carla.
    2 - Task: Decrease the driveability of the scenario by enhancing it with
    more details and complexity, using only methods that are part of the
    official Carla API, version 0.9.15.
    3 - Input, the Python specification for the scenario: {python_carla_scenario_raw}
    4 - Reasoning: Think step by step about how to make the scenario more complex and less driveable, considering possible obstacles, traffic, weather, and other factors using only the official Carla API.
    5 - Output: Only output the enhanced scenario code in Python Carla scenario format, with no additional text or explanation.
    """,
]


def name_to_prompt_idx(name: str) -> int:
    mapping = {
        "basic": 0,
        "no_explanation": 1,
        "no_explanation_strict": 2,
        "cot": 3,
        "cot_strict_methods_in_file": 4,
        "cot_strict_carla_api": 5,
    }
    return mapping.get(name, 0)


def get_prompt_for_python_scenario_enhancement(python_carla_scenario_raw: str, prompt_name: str) -> str:
    prompt_idx = name_to_prompt_idx(prompt_name)
    return PROMPTS[prompt_idx](python_carla_scenario_raw)
