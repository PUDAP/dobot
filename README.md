# Dobot

Monorepo for the left and right Dobot M1 Pro drivers and PUDA edge services.

## Packages

- `driver` — shared `dobot-drivers` package exporting `M1ProLeft` and `M1ProRight`
- `dobot-left-edge` — PUDA edge service for machine `dobot-left`
- `dobot-right-edge` — PUDA edge service for machine `dobot-right`

The edge services depend on the shared driver workspace package. Driver tests
and development notebooks are kept under `driver/tests`.

## Configure

Create an environment file for each edge:

```powershell
Copy-Item dobot-left-edge\.env.example dobot-left-edge\.env
Copy-Item dobot-right-edge\.env.example dobot-right-edge\.env
```

Set these values in each `.env`:

- `MACHINE_ID` — must be unique (`dobot-left` or `dobot-right`)
- `NATS_SERVERS` — comma-separated NATS server URLs
- `DOBOT_IP` — IP address of the corresponding robot
- `AUTO_HOME_ON_STARTUP` — send startup controller commands (`true` or `false`)

The example files use `192.168.2.6` for the left robot and `192.168.2.7` for
the right robot. Update them to match the actual controllers.

## Run on Windows

```powershell
cd dobot-left-edge
.\start_edge.bat
```

In another terminal:

```powershell
cd dobot-right-edge
.\start_edge.bat
```

## Run with uv

From the repository root:

```powershell
uv sync --all-packages
uv run --package dobot-left-edge python dobot-left-edge\main.py
uv run --package dobot-right-edge python dobot-right-edge\main.py
```

## Run with Docker

```powershell
docker compose -f dobot-left-edge\compose.yml up -d --build
docker compose -f dobot-right-edge\compose.yml up -d --build
```

View logs:

```powershell
docker compose -f dobot-left-edge\compose.yml logs -f
docker compose -f dobot-right-edge\compose.yml logs -f
```

Stop:

```powershell
docker compose -f dobot-left-edge\compose.yml down
docker compose -f dobot-right-edge\compose.yml down
```

## Test

```powershell
uv run --package dobot-drivers python -m unittest discover driver\tests -p "test_left_*.py"
uv run --package dobot-drivers python -m unittest discover driver\tests -p "test_right_*.py"
```
