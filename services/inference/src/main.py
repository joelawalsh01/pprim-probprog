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
    reasoning_steps = []
    summary = ""

    vp = posterior.get("velocity_persistence", {})
    ld = posterior.get("lateral_damping", {})
    fm = posterior.get("force_magnitude", {})
    mass = posterior.get("mass", {})

    # --- velocity_persistence ---
    if vp:
        vp_mean = vp.get("mean", 0.5)
        vp_std = vp.get("std", 0.0)
        vp_prior = "Beta(2, 2) — centered at 0.5, meaning we start with no assumption about whether force replaces or adds to velocity"

        reasoning_steps.append({
            "parameter": "velocity_persistence",
            "prior": vp_prior,
            "prior_mean": 0.5,
            "posterior_mean": round(vp_mean, 4),
            "posterior_std": round(vp_std, 4),
            "shift": round(vp_mean - 0.5, 4),
            "rule": (
                "This parameter controls what happens to existing velocity when a new force is applied. "
                "At 0, the force completely replaces the current velocity (the ball forgets it was moving horizontally). "
                "At 1, the force adds to the current velocity (Newtonian superposition). "
                "In the model's forward simulation, horizontal velocity at each timestep is multiplied by "
                "velocity_persistence — so values near 0 kill horizontal motion instantly."
            ),
            "thresholds": [
                {"condition": "mean < 0.3", "triggered": vp_mean < 0.3, "label": "Force replaces velocity (naive / 'force as mover' p-prim)"},
                {"condition": "0.3 ≤ mean ≤ 0.7", "triggered": 0.3 <= vp_mean <= 0.7, "label": "Mixed beliefs — partial persistence"},
                {"condition": "mean > 0.7", "triggered": vp_mean > 0.7, "label": "Force adds to velocity (Newtonian / 'force as deflector')"},
            ],
            "conclusion": (
                f"Posterior mean = {vp_mean:.3f} (shifted {'toward 0' if vp_mean < 0.5 else 'toward 1'} from prior mean of 0.5). "
                + (
                    "This is strong evidence for the 'force as mover' p-prim — the person's predicted trajectory is best explained by a model where force replaces existing velocity."
                    if vp_mean < 0.3 else
                    "This suggests beliefs broadly consistent with Newtonian mechanics — force adds to existing motion."
                    if vp_mean > 0.7 else
                    "This suggests a mixed model — the person partially preserves existing velocity but not fully."
                )
            ),
        })

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

    # --- lateral_damping ---
    if ld:
        ld_mean = ld.get("mean", 1.0)
        ld_std = ld.get("std", 0.0)
        ld_prior_mean = 1.65  # exp(0 + 1^2/2) for LogNormal(0,1)
        ld_prior = "LogNormal(0, 1) — broad prior allowing both low damping (motion persists) and high damping (motion dies)"

        reasoning_steps.append({
            "parameter": "lateral_damping",
            "prior": ld_prior,
            "prior_mean": round(ld_prior_mean, 4),
            "posterior_mean": round(ld_mean, 4),
            "posterior_std": round(ld_std, 4),
            "shift": round(ld_mean - ld_prior_mean, 4),
            "rule": (
                "This parameter controls how quickly horizontal velocity decays over time, independent of "
                "velocity_persistence. In the model, horizontal velocity is multiplied by exp(-lateral_damping × dt) "
                "each timestep. High values mean horizontal motion dies quickly even without a new force — "
                "like an implicit belief that objects naturally stop moving (an Aristotelian intuition). "
                "Low values mean motion persists (Newton's first law)."
            ),
            "thresholds": [
                {"condition": "mean < 0.5", "triggered": ld_mean < 0.5, "label": "Low damping — motion persists (Newtonian)"},
                {"condition": "0.5 ≤ mean ≤ 2.0", "triggered": 0.5 <= ld_mean <= 2.0, "label": "Moderate damping"},
                {"condition": "mean > 2.0", "triggered": ld_mean > 2.0, "label": "High damping — motion dies quickly (Aristotelian)"},
            ],
            "conclusion": (
                f"Posterior mean = {ld_mean:.3f}. "
                + (
                    "High damping reinforces the naive picture: the person expects horizontal motion to dissipate, as if objects naturally come to rest unless actively pushed."
                    if ld_mean > 2.0 else
                    "Low damping is consistent with Newton's first law — objects in motion stay in motion."
                    if ld_mean < 0.5 else
                    "Moderate damping — some belief in natural deceleration but not extreme."
                )
            ),
        })

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

    # --- force_magnitude ---
    if fm:
        fm_mean = fm.get("mean", 1.0)
        fm_std = fm.get("std", 0.0)
        fm_prior_mean = 3.08  # exp(1 + 0.5^2/2) for LogNormal(1, 0.5)

        reasoning_steps.append({
            "parameter": "force_magnitude",
            "prior": "LogNormal(1, 0.5) — prior expectation of moderate force",
            "prior_mean": round(fm_prior_mean, 4),
            "posterior_mean": round(fm_mean, 4),
            "posterior_std": round(fm_std, 4),
            "shift": round(fm_mean - fm_prior_mean, 4),
            "rule": (
                "The perceived strength of the applied force. This interacts with mass to determine "
                "vertical acceleration (a = F/m). The inference adjusts this to match the vertical "
                "component of the observed trajectory."
            ),
            "thresholds": [],
            "conclusion": f"Posterior mean = {fm_mean:.3f}. The model found this force magnitude best reproduces the vertical motion in the observed trajectory.",
        })

    # --- mass ---
    if mass:
        m_mean = mass.get("mean", 1.0)
        m_std = mass.get("std", 0.0)
        m_prior_mean = 1.13  # exp(0 + 0.5^2/2) for LogNormal(0, 0.5)

        reasoning_steps.append({
            "parameter": "mass",
            "prior": "LogNormal(0, 0.5) — prior centered near 1 kg",
            "prior_mean": round(m_prior_mean, 4),
            "posterior_mean": round(m_mean, 4),
            "posterior_std": round(m_std, 4),
            "shift": round(m_mean - m_prior_mean, 4),
            "rule": (
                "The perceived mass of the object. Together with force_magnitude, this determines "
                "vertical acceleration. Mass and force are partially degenerate (only their ratio matters "
                "for acceleration), so the posterior may be broad."
            ),
            "thresholds": [],
            "conclusion": f"Posterior mean = {m_mean:.3f}. Combined with force_magnitude, this gives a perceived acceleration of {fm.get('mean', 10.0) / m_mean:.2f} m/s²." if fm else f"Posterior mean = {m_mean:.3f}.",
        })

    # --- summary ---
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
        "reasoning": reasoning_steps,
    }
