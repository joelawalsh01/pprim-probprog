"""MuJoCo simulation engine for the diSessa balls project."""

import numpy as np
import mujoco
from dataclasses import dataclass


@dataclass
class SimConfig:
    duration: float = 2.0
    initial_vx: float = 3.0
    initial_vy: float = 0.0
    initial_vz: float = 0.0
    force_start_time: float = 0.5
    force_magnitude: float = 10.0
    damping: float = 0.0
    frame_skip: int = 10  # Record every Nth step for frames


@dataclass
class TrajectoryPoint:
    t: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float

    def to_dict(self):
        return {
            "t": round(self.t, 6),
            "x": round(self.x, 6),
            "y": round(self.y, 6),
            "z": round(self.z, 6),
            "vx": round(self.vx, 6),
            "vy": round(self.vy, 6),
            "vz": round(self.vz, 6),
        }


def run_simulation(
    mjcf_xml: str,
    config: SimConfig,
) -> list[dict]:
    """Run a MuJoCo simulation and return the trajectory.

    Args:
        mjcf_xml: MJCF XML string defining the scene.
        config: Simulation configuration.

    Returns:
        List of trajectory point dicts.
    """
    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    data = mujoco.MjData(model)

    # Apply damping override if specified
    if config.damping > 0:
        # Find the free joint and set its damping
        for i in range(model.njnt):
            model.jnt_damping[i] = config.damping

    # Set initial velocity
    if model.nq >= 7:  # Free joint: 7 qpos (pos + quat), 6 qvel
        data.qvel[0] = config.initial_vx
        data.qvel[1] = config.initial_vy
        data.qvel[2] = config.initial_vz

    trajectory = []
    step = 0

    while data.time < config.duration:
        # Apply upward force after trigger time
        if data.time >= config.force_start_time and model.nu > 0:
            data.ctrl[0] = config.force_magnitude
        elif model.nu > 0:
            data.ctrl[0] = 0.0

        mujoco.mj_step(model, data)
        step += 1

        # Record trajectory at intervals
        if step % config.frame_skip == 0:
            point = TrajectoryPoint(
                t=data.time,
                x=float(data.qpos[0]),
                y=float(data.qpos[1]),
                z=float(data.qpos[2]),
                vx=float(data.qvel[0]),
                vy=float(data.qvel[1]),
                vz=float(data.qvel[2]),
            )
            trajectory.append(point.to_dict())

    return trajectory


def run_simulation_batch(
    mjcf_xml: str,
    param_sets: list[dict],
    base_config: SimConfig,
) -> list[list[dict]]:
    """Run N simulations with different parameter overrides.

    Args:
        mjcf_xml: MJCF XML string.
        param_sets: List of dicts with parameter overrides
                    (e.g., {"damping": 5.0, "force_magnitude": 15.0}).
        base_config: Base simulation configuration.

    Returns:
        List of trajectories.
    """
    results = []
    for params in param_sets:
        cfg = SimConfig(
            duration=base_config.duration,
            initial_vx=params.get("initial_vx", base_config.initial_vx),
            initial_vy=params.get("initial_vy", base_config.initial_vy),
            initial_vz=params.get("initial_vz", base_config.initial_vz),
            force_start_time=params.get("force_start_time", base_config.force_start_time),
            force_magnitude=params.get("force_magnitude", base_config.force_magnitude),
            damping=params.get("damping", base_config.damping),
            frame_skip=base_config.frame_skip,
        )
        traj = run_simulation(mjcf_xml, cfg)
        results.append(traj)
    return results


def generate_alternative_trajectory(config: SimConfig) -> list[dict]:
    """Generate the alternative trajectory analytically.

    In the alternative model, the ball moves horizontally until the force is applied,
    then moves straight up (force "replaces" velocity).
    """
    dt = config.duration / 200  # 200 points
    trajectory = []
    t = 0.0
    mass = 1.0  # Assume unit mass

    while t < config.duration:
        if t < config.force_start_time:
            # Moving horizontally
            x = config.initial_vx * t
            z = 0.5  # Starting height
            vx = config.initial_vx
            vz = 0.0
        else:
            # Force replaces velocity: ball goes straight up from where it was
            x_at_force = config.initial_vx * config.force_start_time
            dt_after = t - config.force_start_time
            x = x_at_force  # Horizontal motion stops
            accel = config.force_magnitude / mass
            z = 0.5 + 0.5 * accel * dt_after**2
            vx = 0.0
            vz = accel * dt_after

        trajectory.append({
            "t": round(t, 6),
            "x": round(x, 6),
            "y": 0.0,
            "z": round(z, 6),
            "vx": round(vx, 6),
            "vy": 0.0,
            "vz": round(vz, 6),
        })
        t += dt

    return trajectory
