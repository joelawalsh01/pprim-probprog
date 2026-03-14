/**
 * Default Pyro model code strings for each scenario.
 * These are pasted into the Pyro Model > Code tab in the dashboard.
 */

export const SPIRAL_TUBE_PYRO_MODEL = `import torch
import pyro
import pyro.distributions as dist

def physics_model(observed_trajectory, dt=0.01):
    """Probabilistic model for curvilinear impetus.

    Parameters being inferred:
    - path_curvature_persistence: 0 = straight tangent exit (Newtonian), 1 = continues curving
    - exit_speed: speed of ball at channel exit
    - sigma: observation noise
    """
    n_steps = observed_trajectory.shape[0]

    # Priors over cognitive physics parameters
    path_curvature_persistence = pyro.sample(
        "path_curvature_persistence", dist.Beta(2.0, 2.0)
    )
    exit_speed = pyro.sample(
        "exit_speed", dist.LogNormal(0.5, 0.5)
    )
    sigma = pyro.sample(
        "sigma", dist.LogNormal(-2.0, 1.0)
    )

    # The ball exits a curved channel (quarter-circle, radius=1m) at the top
    # Exit point: (0, 1.05) — top of the arc
    # Newtonian tangent direction at exit: purely horizontal (+x)
    # Alternative: ball continues curving as if the tube imparted angular motion

    exit_x = 0.0
    exit_z = 1.05
    channel_exit_step = n_steps // 4  # Ball is in the channel for the first 25%

    # Channel is a quarter-circle from (-1, 0.5) to (0, 1.05), radius ~1m
    channel_radius = 1.0
    channel_center_x = 0.0
    channel_center_z = 0.05  # center of the arc

    positions = torch.zeros(n_steps, 2)  # x, z

    for i in range(n_steps):
        if i < channel_exit_step:
            # Ball travels through curved channel (quarter arc)
            frac = i / channel_exit_step
            angle = 3.14159 / 2.0 * (1.0 - frac)  # from pi/2 (left) to 0 (top)
            cx = channel_center_x - channel_radius * torch.cos(torch.tensor(angle))
            cz = channel_center_z + channel_radius * torch.sin(torch.tensor(angle))
            positions[i, 0] = cx
            positions[i, 1] = cz
        else:
            # After exit: interpolate between straight tangent and continued curve
            t_after = (i - channel_exit_step) * dt

            # Newtonian: straight tangent (horizontal, +x direction)
            straight_x = exit_x + exit_speed * t_after
            straight_z = exit_z

            # Alternative: continues curving (extends the arc beyond the exit)
            continued_angle = -exit_speed * t_after / channel_radius
            curve_x = exit_x + channel_radius * torch.sin(-continued_angle)
            curve_z = exit_z + channel_radius * (1.0 - torch.cos(continued_angle))

            # Blend based on path_curvature_persistence
            x = (1.0 - path_curvature_persistence) * straight_x + path_curvature_persistence * curve_x
            z = (1.0 - path_curvature_persistence) * straight_z + path_curvature_persistence * curve_z
            positions[i, 0] = x
            positions[i, 1] = z

    # Likelihood
    pyro.sample(
        "obs",
        dist.Normal(positions, sigma).to_event(2),
        obs=observed_trajectory,
    )

    return positions
`;

export const FRICTIONLESS_PUSH_PYRO_MODEL = `import torch
import pyro
import pyro.distributions as dist

def physics_model(observed_trajectory, dt=0.01):
    """Probabilistic model for continuous force / dying away.

    Parameters being inferred:
    - intrinsic_damping: 0 = constant velocity (Newtonian), high = motion dies away
    - push_force: magnitude of the initial push
    - mass: perceived mass of the object
    - sigma: observation noise
    """
    n_steps = observed_trajectory.shape[0]

    # Priors over cognitive physics parameters
    intrinsic_damping = pyro.sample(
        "intrinsic_damping", dist.LogNormal(0.0, 1.0)
    )
    push_force = pyro.sample(
        "push_force", dist.LogNormal(1.0, 0.5)
    )
    mass = pyro.sample(
        "mass", dist.LogNormal(0.0, 0.5)
    )
    sigma = pyro.sample(
        "sigma", dist.LogNormal(-2.0, 1.0)
    )

    # Object starts at rest, receives a brief push for the first 10% of the simulation,
    # then travels freely on a frictionless surface
    push_end_step = n_steps // 10

    positions = torch.zeros(n_steps, 2)  # x, z (z is constant for top-down view)
    vx = 0.0
    x = -3.0
    z = 0.5  # constant height

    for i in range(n_steps):
        if i < push_end_step:
            # Active push phase
            accel = push_force / mass
            vx = vx + accel * dt
        else:
            # Free phase: Newtonian = constant velocity, alternative = decays
            vx = vx * torch.exp(-intrinsic_damping * dt)

        x = x + vx * dt
        positions[i, 0] = x
        positions[i, 1] = z  # no vertical motion

    # Likelihood
    pyro.sample(
        "obs",
        dist.Normal(positions, sigma).to_event(2),
        obs=observed_trajectory,
    )

    return positions
`;

export const GALILEO_DROP_PYRO_MODEL = `import torch
import pyro
import pyro.distributions as dist

def physics_model(observed_trajectory, dt=0.01):
    """Probabilistic model for heavier-falls-faster (Ohm's p-prim).

    This models a single ball's drop. The key parameter mass_gravity_coupling
    controls whether the ball's mass affects its fall rate.

    When mass_gravity_coupling = 0: a = g (Newtonian, mass-independent)
    When mass_gravity_coupling = 1: a = g * (1 + mass_ratio), heavier falls faster

    Parameters being inferred:
    - mass_gravity_coupling: 0 = mass-independent (Newtonian), 1 = heavier falls faster
    - g_effective: effective gravitational acceleration
    - sigma: observation noise
    """
    n_steps = observed_trajectory.shape[0]

    # Priors over cognitive physics parameters
    mass_gravity_coupling = pyro.sample(
        "mass_gravity_coupling", dist.Beta(2.0, 2.0)
    )
    g_effective = pyro.sample(
        "g_effective", dist.LogNormal(2.3, 0.3)
    )
    sigma = pyro.sample(
        "sigma", dist.LogNormal(-2.0, 1.0)
    )

    # Model the heavy ball (mass=5kg) and light ball (mass=0.5kg)
    # Trajectory has 4 columns: heavy_x, heavy_z, light_x, light_z
    # But we use 2-column format: x = heavy_z, z = light_z (both vertical drops)

    heavy_mass = 5.0
    light_mass = 0.5
    reference_mass = 1.0  # reference mass for coupling calculation

    positions = torch.zeros(n_steps, 2)  # col0 = heavy_z, col1 = light_z

    heavy_z = 5.0
    light_z = 5.0
    heavy_vz = 0.0
    light_vz = 0.0

    for i in range(n_steps):
        # Newtonian: both accelerate at g regardless of mass
        # Alternative: acceleration scales with mass (heavier = faster)
        heavy_accel = g_effective * (1.0 + mass_gravity_coupling * (heavy_mass / reference_mass - 1.0))
        light_accel = g_effective * (1.0 + mass_gravity_coupling * (light_mass / reference_mass - 1.0))

        heavy_vz = heavy_vz - heavy_accel * dt
        light_vz = light_vz - light_accel * dt

        heavy_z = heavy_z + heavy_vz * dt
        light_z = light_z + light_vz * dt

        positions[i, 0] = heavy_z
        positions[i, 1] = light_z

    # Likelihood
    pyro.sample(
        "obs",
        dist.Normal(positions, sigma).to_event(2),
        obs=observed_trajectory,
    )

    return positions
`;
