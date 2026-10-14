FROM python:3.14-slim-bookworm

WORKDIR /app
ENV UV_COMPILE_BYTECODE=1

RUN apt-get update && apt-get install -y build-essential \
  && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv
COPY pyproject.toml uv.lock ./
COPY driver/ ./driver/
COPY dobot-left-edge/ ./dobot-left-edge/
COPY dobot-right-edge/ ./dobot-right-edge/

WORKDIR /app/dobot-left-edge
RUN uv sync --frozen --no-dev --package dobot-left-edge

ENV PYTHONPATH=/app/dobot-left-edge:/app/driver/src
CMD ["uv", "run", "python", "main.py"]
