"""Pyro probabilistic models for physics prior extraction."""

import torch
import pyro
import pyro.distributions as dist
import numpy as np


DEFAULT_MODEL_CODE = '''
import torch
import pyro
import pyro.distributions as dist

def physics_model(observed_trajectory, dt=0.01):
    """Probabilistic model of alternative physics beliefs.

    Parameters being inferred:
    - velocity_persistence: 0 = force replaces velocity (alternative), 1 = force adds (Newtonian)
    - lateral_damping: how quickly horizontal velocity decays (high = alternative belief)
    - force_magnitude: perceived strength of the applied force
    - mass: perceived mass of the object
    - sigma: observation noise
    """
    n_steps = observed_trajectory.shape[0]

    # Priors over "cognitive physics" parameters
    velocity_persistence = pyro.sample(
        "velocity_persistence", dist.Beta(2.0, 2.0)
    )
    lateral_damping = pyro.sample(
        "lateral_damping", dist.LogNormal(0.0, 1.0)
    )
    force_magnitude = pyro.sample(
        "force_magnitude", dist.LogNormal(1.0, 0.5)
    )
    mass = pyro.sample(
        "mass", dist.LogNormal(0.0, 0.5)
    )
    sigma = pyro.sample(
        "sigma", dist.LogNormal(-2.0, 1.0)
    )

    # Forward simulate with these cognitive physics parameters
    initial_vx = 3.0
    force_start_step = n_steps // 4  # Force starts at 25% through

    positions = torch.zeros(n_steps, 2)  # x, z
    vx = initial_vx
    vz = 0.0
    x = -2.0
    z = 0.5

    for i in range(n_steps):
        if i >= force_start_step:
            # Apply force: velocity_persistence controls whether force adds or replaces
            accel_z = force_magnitude / mass
            vz = vz + accel_z * dt

            # Lateral damping + persistence: how much horizontal velocity survives
            vx = vx * velocity_persistence * torch.exp(-lateral_damping * dt)

        x = x + vx * dt
        z = z + vz * dt
        positions[i, 0] = x
        positions[i, 1] = z

    # Likelihood
    pyro.sample(
        "obs",
        dist.Normal(positions, sigma).to_event(2),
        obs=observed_trajectory,
    )

    return positions
'''.strip()


def build_forward_model(
    velocity_persistence: float,
    lateral_damping: float,
    force_magnitude: float,
    mass: float,
    n_steps: int = 200,
    dt: float = 0.01,
    initial_vx: float = 3.0,
    force_start_frac: float = 0.25,
) -> torch.Tensor:
    """Run forward simulation with given cognitive physics parameters.

    Returns positions as (n_steps, 2) tensor of [x, z].
    """
    force_start_step = int(n_steps * force_start_frac)
    positions = torch.zeros(n_steps, 2)

    vx = initial_vx
    vz = 0.0
    x = -2.0
    z = 0.5

    for i in range(n_steps):
        if i >= force_start_step:
            accel_z = force_magnitude / mass
            vz = vz + accel_z * dt
            vx = vx * velocity_persistence * torch.exp(torch.tensor(-lateral_damping * dt))

        x = x + vx * dt
        z = z + vz * dt
        positions[i, 0] = x
        positions[i, 1] = z

    return positions


def physics_model(observed_trajectory: torch.Tensor, dt: float = 0.01):
    """Pyro probabilistic model of alternative physics beliefs."""
    n_steps = observed_trajectory.shape[0]

    velocity_persistence = pyro.sample(
        "velocity_persistence", dist.Beta(2.0, 2.0)
    )
    lateral_damping = pyro.sample(
        "lateral_damping", dist.LogNormal(0.0, 1.0)
    )
    force_magnitude = pyro.sample(
        "force_magnitude", dist.LogNormal(1.0, 0.5)
    )
    mass = pyro.sample(
        "mass", dist.LogNormal(0.0, 0.5)
    )
    sigma = pyro.sample(
        "sigma", dist.LogNormal(-2.0, 1.0)
    )

    force_start_step = n_steps // 4
    positions = torch.zeros(n_steps, 2)

    vx = torch.tensor(3.0)
    vz = torch.tensor(0.0)
    x = torch.tensor(-2.0)
    z = torch.tensor(0.5)

    for i in range(n_steps):
        if i >= force_start_step:
            accel_z = force_magnitude / mass
            vz = vz + accel_z * dt
            vx = vx * velocity_persistence * torch.exp(-lateral_damping * dt)

        x = x + vx * dt
        z = z + vz * dt
        positions[i, 0] = x
        positions[i, 1] = z

    pyro.sample(
        "obs",
        dist.Normal(positions, sigma).to_event(2),
        obs=observed_trajectory,
    )

    return positions


def generate_alternative_trajectory_tensor(
    n_steps: int = 200,
    dt: float = 0.01,
    initial_vx: float = 3.0,
    force_magnitude: float = 10.0,
    mass: float = 1.0,
) -> torch.Tensor:
    """Generate the alternative trajectory as a tensor (ball goes straight up).

    Returns (n_steps, 2) tensor of [x, z].
    """
    force_start_step = n_steps // 4
    positions = torch.zeros(n_steps, 2)

    x = -2.0
    z = 0.5
    vz = 0.0

    for i in range(n_steps):
        if i < force_start_step:
            x += initial_vx * dt
        else:
            accel_z = force_magnitude / mass
            vz += accel_z * dt
            z += vz * dt
            # x stays fixed (alternative belief: force replaces velocity)

        positions[i, 0] = x
        positions[i, 1] = z

    return positions


def generate_newtonian_trajectory_tensor(
    n_steps: int = 200,
    dt: float = 0.01,
    initial_vx: float = 3.0,
    force_magnitude: float = 10.0,
    mass: float = 1.0,
) -> torch.Tensor:
    """Generate the Newtonian trajectory as a tensor (ball curves diagonally).

    Returns (n_steps, 2) tensor of [x, z].
    """
    force_start_step = n_steps // 4
    positions = torch.zeros(n_steps, 2)

    x = -2.0
    z = 0.5
    vz = 0.0

    for i in range(n_steps):
        if i >= force_start_step:
            accel_z = force_magnitude / mass
            vz += accel_z * dt

        x += initial_vx * dt
        z += vz * dt
        positions[i, 0] = x
        positions[i, 1] = z

    return positions
