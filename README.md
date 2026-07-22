# dobot

Dobot M1 Pro driver and edge service.

## What It Does

- Runs the dobot machine edge service.
- Connects to NATS and translates commands into machine actions.
- Communicates with the dobot device over the network (IP).

## Prerequisites

- Docker and Docker Compose installed
- Python 3.14+ and `uv` (for baremetal mode)
- dobot device reachable on the network

## Environment Setup

From repo root:

```bash
cp .env.example .env
```

Edit `.env` and configure:

- `MACHINE_ID`
- `NATS_SERVERS`
- `DOBOT_IP`

## Run With Docker (Recommended)

All commands below are run from repo root.

Build and start:

```bash
docker compose up -d --build
```

View logs:

```bash
docker compose logs -f
```

Stop:

```bash
docker compose down
```

## Run Baremetal (uv)

From repo root:

```bash
uv sync
uv run python main.py
```

## Build and Push Image (GHCR)

Login:

```bash
echo $GITHUB_TOKEN | docker login ghcr.io -u USERNAME --password-stdin
```

Build:

```bash
docker compose build
```

Push:

```bash
docker push ghcr.io/PUDAP/dobot-edge:latest
```

Or with Compose:

```bash
docker compose push
```
