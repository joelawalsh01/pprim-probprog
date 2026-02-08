FROM python:3.11-slim

# Install system dependencies for MuJoCo + OSMesa software rendering
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dev \
    libosmesa6-dev \
    libglfw3-dev \
    patchelf \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /app

# Copy project files
COPY pyproject.toml .
RUN uv sync --no-dev

COPY src/ src/

ENV MUJOCO_GL=osmesa

EXPOSE 8001
CMD ["uv", "run", "uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8001"]
