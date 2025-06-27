# master-hefe

Monorepo for code of [my master thesis](https://github.com/orjahren/master).

## Running

`podman-compose up`

NOTE: Hvis man ikke remover kan man ende opp med å kjøre gamle containers selv
om nye images har blitt bygget

` podman rm odin thor && podman-compose up`

## Dependencies

TODO: Write requirements.txt file

- FastAPI
  - `pip install fastapi`
  - `pip install "fastapi[standard]`
- Carla

## Components

### Thor

Running test cases

### Odin

Performing LLM enhancement

### Loki

Frontend/interaction
