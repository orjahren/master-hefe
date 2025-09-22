
import requests


OLLAMA_API_URL = "http://localhost:11434"


def api_is_up():
    try:
        response = requests.get(OLLAMA_API_URL)
        return response.status_code == 200
    except requests.ConnectionError:
        return False


# ollama models
def get_ollama_models():
    try:
        response = requests.get(f"{OLLAMA_API_URL}/api/tags")
        if response.status_code == 200:
            return response.json()["models"]
        else:
            print(f"Failed to get models: {response.status_code}")
            return []
    except requests.ConnectionError:
        print("Failed to connect to the API.")
        return []
