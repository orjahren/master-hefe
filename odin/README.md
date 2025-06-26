# Rest API - performing LLM enhancement - Odin

- “Fast API”, Python
- Take a base test case as body
- Have some prompt repository
- Apply prompts with LLMs
- Must integrate with LLM. Either locally (Ollama) or remote (some API)
- Look into good LLM agnostic transition layer. E.g. Aisuite
- https://github.com/andrewyng/aisuite
- Should use same UUIDs as outlined above, but suffixed with e.g. “pure” and “tainted”
- Containerized. Docker compose?

## Kjøre

`podman run -it -p 4000:4000 odin`

NB: Støtter ikke live reload

## Bygge

`podman build  . -t odin ` ( i denne mappen)
