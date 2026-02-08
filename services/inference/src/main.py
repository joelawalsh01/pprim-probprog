"""FastAPI application for the Pyro inference service."""

import torch
import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from .models import (
    DEFAULT_MODEL_CODE,
    generate_naive_trajectory_tensor,
    generate_newtonian_trajectory_tensor,
)
from .inference import (
    run_svi,
    run_mcmc,
    sample_prior_predictive,
    sample_posterior_predictive,
)

app = FastAPI(title="DiSessa Inference", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Store latest posterior for posterior-predictive sampling
_latest_posterior: dict | None = None


class InferRequest(BaseModel):
    observed_trajectory: list[dict] | None = None
    trajectory_type: str = "naive"  # "naive" or "custom"
    model_code: str | None = None
    method: str = "svi"
    num_steps: int = 1000
    num_samples: int = 200
    learning_rate: float = 0.01
    n_trajectory_steps: int = 200
    dt: float = 0.01


class PriorPredictiveRequest(BaseModel):
    num_samples: int = 50
    n_steps: int = 200
    dt: float = 0.01


class PosteriorPredictiveRequest(BaseModel):
    num_samples: int = 50
    n_steps: int = 200
    dt: float = 0.01


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/models")
def list_models():
    """List available pre-built models."""
    return {
        "models": [
            {
                "name": "cognitive_physics",
                "description": "Infers velocity_persistence, lateral_damping, force_magnitude, mass from observed trajectory",
                "parameters": [
                    {"name": "velocity_persistence", "prior": "Beta(2, 2)", "interpretation": "0=force replaces velocity, 1=force adds"},
                    {"name": "lateral_damping", "prior": "LogNormal(0, 1)", "interpretation": "Rate of horizontal velocity decay"},
                    {"name": "force_magnitude", "prior": "LogNormal(1, 0.5)", "interpretation": "Perceived force strength"},
                    {"name": "mass", "prior": "LogNormal(0, 0.5)", "interpretation": "Perceived object mass"},
                    {"name": "sigma", "prior": "LogNormal(-2, 1)", "interpretation": "Observation noise"},
                ],
                "code": DEFAULT_MODEL_CODE,
            }
        ]
    }


@app.post("/infer")
def infer(req: InferRequest):
    """Run probabilistic inference to extract physics priors."""
    global _latest_posterior

    # Generate or use provided observed trajectory
    if req.observed_trajectory and req.trajectory_type == "custom":
        # Convert custom trajectory to tensor
        xz = [[p.get("x", 0), p.get("z", 0)] for p in req.observed_trajectory]
        observed = torch.tensor(xz, dtype=torch.float32)
    else:
        # Generate naive trajectory (ball goes straight up)
        observed = generate_naive_trajectory_tensor(
            n_steps=req.n_trajectory_steps,
            dt=req.dt,
        )

    # Also generate the Newtonian trajectory for comparison
    newtonian = generate_newtonian_trajectory_tensor(
        n_steps=req.n_trajectory_steps,
        dt=req.dt,
    )

    try:
        if req.method == "svi":
            result = run_svi(
                observed,
                num_steps=req.num_steps,
                lr=req.learning_rate,
            )
        elif req.method == "mcmc":
            result = run_mcmc(
                observed,
                num_samples=req.num_samples,
                warmup_steps=max(50, req.num_samples // 2),
            )
        else:
            raise HTTPException(status_code=400, detail=f"Unknown method: {req.method}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Inference error: {str(e)}")

    # Store posterior for later predictive sampling
    _latest_posterior = result.get("posterior", {})

    # Add trajectory data to result
    observed_list = []
    newtonian_list = []
    for i in range(observed.shape[0]):
        t = round(i * req.dt, 6)
        observed_list.append({"t": t, "x": round(float(observed[i, 0]), 6), "z": round(float(observed[i, 1]), 6)})
        newtonian_list.append({"t": t, "x": round(float(newtonian[i, 0]), 6), "z": round(float(newtonian[i, 1]), 6)})

    result["observed_trajectory"] = observed_list
    result["newtonian_trajectory"] = newtonian_list
    result["model_code"] = req.model_code or DEFAULT_MODEL_CODE

    # Generate interpretation
    posterior = result.get("posterior", {})
    result["interpretation"] = generate_interpretation(posterior)

    return result


@app.post("/prior-predictive")
def prior_predictive(req: PriorPredictiveRequest):
    """Sample trajectories from prior distributions."""
    trajectories = sample_prior_predictive(
        num_samples=req.num_samples,
        n_steps=req.n_steps,
        dt=req.dt,
    )
    return {"trajectories": trajectories, "num_samples": len(trajectories)}


@app.post("/posterior-predictive")
def posterior_predictive(req: PosteriorPredictiveRequest):
    """Sample trajectories from posterior distributions."""
    if _latest_posterior is None:
        raise HTTPException(status_code=400, detail="No posterior available. Run /infer first.")

    trajectories = sample_posterior_predictive(
        _latest_posterior,
        num_samples=req.num_samples,
        n_steps=req.n_steps,
        dt=req.dt,
    )
    return {"trajectories": trajectories, "num_samples": len(trajectories)}


def generate_interpretation(posterior: dict) -> dict:
    """Generate plain-English interpretation of posterior results."""
    findings = []
    summary = ""

    vp = posterior.get("velocity_persistence", {})
    ld = posterior.get("lateral_damping", {})
    fm = posterior.get("force_magnitude", {})
    mass = posterior.get("mass", {})

    if vp:
        vp_mean = vp.get("mean", 0.5)
        if vp_mean < 0.3:
            findings.append(
                f"velocity_persistence is low ({vp_mean:.3f}): This person implicitly believes "
                "that an applied force *replaces* the existing velocity rather than adding to it. "
                "This is the classic naive physics misconception."
            )
        elif vp_mean > 0.7:
            findings.append(
                f"velocity_persistence is high ({vp_mean:.3f}): This person's beliefs align "
                "with Newtonian mechanics — force adds to existing velocity."
            )
        else:
            findings.append(
                f"velocity_persistence is moderate ({vp_mean:.3f}): This person partially "
                "believes force replaces velocity but retains some Newtonian intuition."
            )

    if ld:
        ld_mean = ld.get("mean", 1.0)
        if ld_mean > 2.0:
            findings.append(
                f"lateral_damping is high ({ld_mean:.3f}): The person believes horizontal "
                "motion dissipates quickly, as if objects naturally stop moving unless pushed."
            )
        elif ld_mean < 0.5:
            findings.append(
                f"lateral_damping is low ({ld_mean:.3f}): Consistent with Newton's first law — "
                "objects in motion stay in motion."
            )

    if vp and vp.get("mean", 0.5) < 0.3:
        summary = (
            "A person expecting this trajectory implicitly believes that force replaces "
            "velocity rather than adding to it. When the upward force is applied, they "
            "expect the horizontal motion to stop and the ball to go straight up. This "
            "is the 'force as a mover' misconception identified by diSessa — treating "
            "force as setting velocity directly rather than causing acceleration."
        )
    else:
        summary = (
            "The inferred physics priors suggest this person's mental model partially "
            "or fully incorporates Newtonian mechanics."
        )

    return {
        "findings": findings,
        "summary": summary,
    }
