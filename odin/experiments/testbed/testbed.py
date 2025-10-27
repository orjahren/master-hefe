

import os
from odin.llm_api_interfaces.gemini import execute_gemini_model
from odin.llm_api_interfaces.ollama import api_is_up,  execute_ollama_model, get_ollama_models
from odin.scenario_utils import get_available_scenarios, get_enhanced_scenario_name, save_enhanced_scenario, scenario_path_to_string
from prompts import PromptName, get_prompt_for_python_scenario_enhancement

SCENARIO_REPOSITORY_PATH = "odin/experiments/testbed/scenarios"


def model_name_to_function(model_name: str):
    if "mistral" in model_name:
        return execute_ollama_model
    elif "gemini" in model_name:
        return execute_gemini_model
    else:
        raise ValueError(f"Model {model_name} is not supported.")


def enhance_scenario_with_llm(scenario_description: str, model_name: str, prompt_name: PromptName, scenario_name: str, specific_metric: str) -> str:
    if use_ollama and not api_is_up():
        raise ConnectionError("OLLAMA API is not reachable.")

    prompt = get_prompt_for_python_scenario_enhancement(
        scenario_description, prompt_name, scenario_name, specific_metric)

    print("Prompt to be sent to the model:")
    print(prompt[:100])

    execute_model = model_name_to_function(model_name)
    if execute_model is None:
        raise ValueError(f"Model {model_name} is not supported.")

    response = execute_model(model_name, prompt)
    if response is None:
        print("No response received from the model. RIP RIP RIP ")
        raise RuntimeError("Failed to get a response from the OLLAMA model.")

    return response


use_ollama = False

if __name__ == "__main__":
    if use_ollama:
        if api_is_up():
            print("API is up and running!")
        else:
            print("API is not reachable.")
            exit(1)
        models = get_ollama_models()
        if models:
            print("Available models:")
            for model in models:
                print(f"- {model['name']}")
                print(f"\t{model}")
        else:
            print("No models found or failed to retrieve models.")

    scenarios = get_available_scenarios(SCENARIO_REPOSITORY_PATH)
    print("Available scenarios:")
    for scenario in scenarios:
        print(f"- {scenario}")

    ###

    # TEST CONFIG

    ###

    # scenario = "cut_in.py"
    # scenario = "junction.py"
    # scenario = "follow.py"
    # scenario = "route_obstacles.py"

    # prompt_name: PromptName = "cot_strict_carla_api"
    # prompt_name: PromptName = "minimal_changes_specific_metric"

    # prompt_name: PromptName = "no_explanation"
    prompt_name: PromptName = "minimal_changes_shared_file_specific_metric"

    model_name = "gemini-2.5-flash"
    # scenario_name = "Accident"
    # scenario_name = "follow.py"
    # scenario_name = "cut_in.py"
    # scenario_name = "route_obstacles.py"
    scenario_file_name = "route_obstacles.py"
    scenario_name = "Accident"

    metric = "Jerk"

    scenario_path = os.path.join(SCENARIO_REPOSITORY_PATH, scenario_file_name)
    scenario_description = scenario_path_to_string(scenario_path)
    print("Original scenario description:")
    print(scenario_description[:100])

    enhanced_scenario = enhance_scenario_with_llm(
        scenario_description, model_name, prompt_name, scenario_name, metric)

    output_path = SCENARIO_REPOSITORY_PATH + "/" + get_enhanced_scenario_name(
        SCENARIO_REPOSITORY_PATH, scenario_file_name.replace(".py", "")) + ".py"
    print(f"Saving enhanced scenario to: {output_path}")
    save_enhanced_scenario(enhanced_scenario, output_path)

    print("Enhanced scenario description:")
    print(enhanced_scenario)
