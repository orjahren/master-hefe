# master-hefe

Monorepo for code of [my master thesis](https://github.com/orjahren/master).

## Running

`podman-compose up`

`sh loki/liveness.sh`

---

NOTE: Hvis man ikke remover kan man ende opp med å kjøre gamle containers selv
om nye images har blitt bygget

` podman rm odin thor && podman-compose up`

Bedre å bruke `podman-compose down`? Litt usikker på hva den gjør men den ser ut
til å RMe ting

## Dependencies

TODO: Write requirements.txt file

- Standard UNIX utils
  - Curl
- Podman (or Docker, YMMW)
  - Podman-compose
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
