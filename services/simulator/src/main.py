"""FastAPI application for the MuJoCo simulator service."""

import os
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from .simulation import SimConfig, run_simulation, run_simulation_batch, generate_naive_trajectory
from .renderer import render_frames

app = FastAPI(title="DiSessa Simulator", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

EXAMPLES_DIR = Path(__file__).parent / "examples"


class SimulateRequest(BaseModel):
    mjcf_xml: str
    duration: float = 2.0
    initial_vx: float = 3.0
    initial_vy: float = 0.0
    initial_vz: float = 0.0
    force_start_time: float = 0.5
    force_magnitude: float = 10.0
    damping: float = 0.0
    render_frames: bool = True
    frame_width: int = 640
    frame_height: int = 480
    camera_name: str | None = "side"
    max_frames: int = 60
    include_naive: bool = True


class SimulateBatchRequest(BaseModel):
    mjcf_xml: str
    param_sets: list[dict] = Field(default_factory=list)
    duration: float = 2.0
    initial_vx: float = 3.0
    force_start_time: float = 0.5
    force_magnitude: float = 10.0


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/examples")
def list_examples():
    """List available example MJCF files."""
    examples = []
    for f in EXAMPLES_DIR.glob("*.xml"):
        examples.append({"name": f.stem, "filename": f.name})
    return {"examples": examples}


@app.get("/examples/{name}")
def get_example(name: str):
    """Return the MJCF XML for an example."""
    path = EXAMPLES_DIR / f"{name}.xml"
    if not path.exists():
        raise HTTPException(status_code=404, detail=f"Example '{name}' not found")
    return {"name": name, "mjcf_xml": path.read_text()}


@app.post("/simulate")
def simulate(req: SimulateRequest):
    """Run a MuJoCo simulation and return trajectory + optional frames."""
    config = SimConfig(
        duration=req.duration,
        initial_vx=req.initial_vx,
        initial_vy=req.initial_vy,
        initial_vz=req.initial_vz,
        force_start_time=req.force_start_time,
        force_magnitude=req.force_magnitude,
        damping=req.damping,
    )

    try:
        trajectory = run_simulation(req.mjcf_xml, config)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Simulation error: {str(e)}")

    result = {
        "trajectory": trajectory,
        "metadata": {
            "duration": req.duration,
            "num_points": len(trajectory),
            "initial_vx": req.initial_vx,
            "force_start_time": req.force_start_time,
            "force_magnitude": req.force_magnitude,
            "damping": req.damping,
        },
    }

    if req.render_frames:
        try:
            frames = render_frames(
                req.mjcf_xml, config,
                width=req.frame_width,
                height=req.frame_height,
                camera_name=req.camera_name,
                max_frames=req.max_frames,
            )
            result["frames"] = frames
        except Exception as e:
            result["frames"] = []
            result["render_error"] = str(e)

    if req.include_naive:
        result["naive_trajectory"] = generate_naive_trajectory(config)

    return result


@app.post("/simulate-batch")
def simulate_batch(req: SimulateBatchRequest):
    """Run N simulations with different parameter sets."""
    config = SimConfig(
        duration=req.duration,
        initial_vx=req.initial_vx,
        force_start_time=req.force_start_time,
        force_magnitude=req.force_magnitude,
    )

    try:
        trajectories = run_simulation_batch(req.mjcf_xml, req.param_sets, config)
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Batch simulation error: {str(e)}")

    return {"trajectories": trajectories, "num_runs": len(trajectories)}
