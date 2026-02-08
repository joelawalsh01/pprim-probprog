"""Pyro inference runners: SVI and MCMC."""

import torch
import pyro
import pyro.distributions as dist
from pyro.infer import SVI, Trace_ELBO, MCMC, NUTS, Predictive
from pyro.infer.autoguide import AutoNormal
import numpy as np

from .models import physics_model, build_forward_model


def run_svi(
    observed_trajectory: torch.Tensor,
    num_steps: int = 1000,
    lr: float = 0.01,
) -> dict:
    """Run Stochastic Variational Inference.

    Args:
        observed_trajectory: (n_steps, 2) tensor of [x, z] positions.
        num_steps: Number of optimization steps.
        lr: Learning rate.

    Returns:
        Dict with posterior summaries, losses, and samples.
    """
    pyro.clear_param_store()

    guide = AutoNormal(physics_model)
    optimizer = pyro.optim.Adam({"lr": lr})
    svi = SVI(physics_model, guide, optimizer, loss=Trace_ELBO())

    losses = []
    for step in range(num_steps):
        loss = svi.step(observed_trajectory)
        losses.append(loss)

    # Sample from posterior
    predictive = Predictive(physics_model, guide=guide, num_samples=200)
    samples = predictive(observed_trajectory)

    # Extract parameter samples — discover names dynamically from samples dict
    posterior_samples = {}
    for name, tensor in samples.items():
        if name == "obs" or name.startswith("_"):
            continue
        vals = tensor.detach().numpy()
        if vals.ndim == 0 or (vals.ndim == 1 and vals.shape[0] == 0):
            continue
        # Flatten multi-dim samples to 1D (e.g. from Predictive)
        if vals.ndim > 1:
            vals = vals.reshape(-1)
        posterior_samples[name] = {
            "samples": vals.tolist(),
            "mean": float(np.mean(vals)),
            "std": float(np.std(vals)),
            "median": float(np.median(vals)),
            "q05": float(np.percentile(vals, 5)),
            "q95": float(np.percentile(vals, 95)),
        }

    return {
        "method": "svi",
        "num_steps": num_steps,
        "losses": [float(l) for l in losses],
        "posterior": posterior_samples,
    }


def run_mcmc(
    observed_trajectory: torch.Tensor,
    num_samples: int = 200,
    warmup_steps: int = 100,
) -> dict:
    """Run MCMC inference using NUTS.

    Args:
        observed_trajectory: (n_steps, 2) tensor of [x, z] positions.
        num_samples: Number of posterior samples.
        warmup_steps: Number of warmup/burn-in steps.

    Returns:
        Dict with posterior summaries and samples.
    """
    pyro.clear_param_store()

    kernel = NUTS(physics_model)
    mcmc = MCMC(kernel, num_samples=num_samples, warmup_steps=warmup_steps)
    mcmc.run(observed_trajectory)

    samples = mcmc.get_samples()

    posterior_samples = {}
    for name, tensor in samples.items():
        if name == "obs" or name.startswith("_"):
            continue
        vals = tensor.detach().numpy()
        if vals.ndim == 0 or (vals.ndim == 1 and vals.shape[0] == 0):
            continue
        if vals.ndim > 1:
            vals = vals.reshape(-1)
        posterior_samples[name] = {
            "samples": vals.tolist(),
            "mean": float(np.mean(vals)),
            "std": float(np.std(vals)),
            "median": float(np.median(vals)),
            "q05": float(np.percentile(vals, 5)),
            "q95": float(np.percentile(vals, 95)),
        }

    return {
        "method": "mcmc",
        "num_samples": num_samples,
        "warmup_steps": warmup_steps,
        "posterior": posterior_samples,
    }


def sample_prior_predictive(
    num_samples: int = 50,
    n_steps: int = 200,
    dt: float = 0.01,
) -> list[list[dict]]:
    """Sample trajectories from the prior.

    Returns list of trajectories, each a list of {t, x, z} dicts.
    """
    trajectories = []

    for _ in range(num_samples):
        vp = float(dist.Beta(2.0, 2.0).sample())
        ld = float(dist.LogNormal(0.0, 1.0).sample())
        fm = float(dist.LogNormal(1.0, 0.5).sample())
        m = float(dist.LogNormal(0.0, 0.5).sample())

        positions = build_forward_model(vp, ld, fm, m, n_steps=n_steps, dt=dt)

        traj = []
        for i in range(n_steps):
            traj.append({
                "t": round(i * dt, 6),
                "x": round(float(positions[i, 0]), 6),
                "z": round(float(positions[i, 1]), 6),
            })
        trajectories.append(traj)

    return trajectories


def sample_posterior_predictive(
    posterior_samples: dict,
    num_samples: int = 50,
    n_steps: int = 200,
    dt: float = 0.01,
) -> list[list[dict]]:
    """Sample trajectories from the posterior.

    Args:
        posterior_samples: Dict mapping parameter names to {"samples": [...]}.
        num_samples: Number of trajectories to generate.

    Returns list of trajectories.
    """
    trajectories = []

    vp_samples = posterior_samples.get("velocity_persistence", {}).get("samples", [0.5])
    ld_samples = posterior_samples.get("lateral_damping", {}).get("samples", [1.0])
    fm_samples = posterior_samples.get("force_magnitude", {}).get("samples", [10.0])
    m_samples = posterior_samples.get("mass", {}).get("samples", [1.0])

    n = min(num_samples, len(vp_samples))

    for i in range(n):
        idx = i % len(vp_samples)
        vp = vp_samples[idx]
        ld = ld_samples[idx % len(ld_samples)]
        fm = fm_samples[idx % len(fm_samples)]
        m = m_samples[idx % len(m_samples)]

        positions = build_forward_model(vp, ld, fm, m, n_steps=n_steps, dt=dt)

        traj = []
        for j in range(n_steps):
            traj.append({
                "t": round(j * dt, 6),
                "x": round(float(positions[j, 0]), 6),
                "z": round(float(positions[j, 1]), 6),
            })
        trajectories.append(traj)

    return trajectories
