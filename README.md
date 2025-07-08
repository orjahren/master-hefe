# master-hefe

Monorepo for code of [my master thesis](https://github.com/orjahren/master).

## Running

1. `make`
1. Start Carla on your host machine
   1. Expects RPC on port 2000
1. `sh loki/liveness.sh`

## Dependencies

TODO: Write requirements.txt file

- Standard UNIX utils
  - `curl`
  - `jq`
  - `make`
- Docker
- Python 3.13 (for running the [loki module](./loki/))
  - requests
  - pika

## Components

### Thor

Running test cases

### Odin

Performing LLM enhancement

### Loki

Frontend/interaction
