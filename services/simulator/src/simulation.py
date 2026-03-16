"""MuJoCo simulation engine for the diSessa balls project."""

import math
import numpy as np
import mujoco
from dataclasses import dataclass, field


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


def _get_model_name(mjcf_xml: str) -> str:
    """Extract the model name from MJCF XML."""
    import re
    m = re.search(r'<mujoco\s+model="([^"]*)"', mjcf_xml)
    return m.group(1) if m else ""


def _spiral_tube_position(t: float, duration: float, speed: float = 2.0):
    """Compute ball position for the spiral tube scenario.

    Phase 1 (first 40% of duration): Ball follows a quarter-circle arc
        Arc center: (0, 0, 0.05), radius 1.0
        From angle pi (left, (-1, 0.5)) to angle pi/2 (top, (0, 1.05))

    Phase 2 (remaining 60%): Ball exits tangentially (straight right)
        Exit point: (0.05, 0, 1.05), exit velocity: (+speed, 0, 0)
    """
    arc_time = duration * 0.4
    arc_center_x = 0.0
    arc_center_z = 0.05
    arc_radius = 1.0

    if t < arc_time:
        # Arc phase: sweep from angle pi to pi/2
        frac = t / arc_time
        angle = math.pi * (1.0 - 0.5 * frac)  # pi → pi/2
        x = arc_center_x + arc_radius * math.cos(angle)
        z = arc_center_z + arc_radius * math.sin(angle)
        # Tangential velocity (perpendicular to radius, counterclockwise)
        arc_speed = (math.pi / 2.0) * arc_radius / arc_time
        vx = arc_speed * math.sin(angle)
        vz = -arc_speed * (-math.cos(angle))  # derivative of sin
        # Correct tangential velocity for counterclockwise motion
        vx = arc_radius * (math.pi / 2.0 / arc_time) * math.sin(angle)
        vz = arc_radius * (math.pi / 2.0 / arc_time) * (-math.cos(angle))
        # At angle pi: vx = 0, vz = +; at angle pi/2: vx = +, vz = 0 ✓
        return x, 0.0, z, vx, 0.0, vz
    else:
        # Straight exit phase: ball exits at (exit_x, exit_z) moving right
        exit_x = arc_center_x + arc_radius * math.cos(math.pi / 2)  # ~0
        exit_z = arc_center_z + arc_radius * math.sin(math.pi / 2)  # ~1.05
        dt_after = t - arc_time
        x = exit_x + speed * dt_after
        z = exit_z
        return x, 0.0, z, speed, 0.0, 0.0


def run_simulation(
    mjcf_xml: str,
    config: SimConfig,
) -> list[dict]:
    """Run a MuJoCo simulation and return the trajectory.

    For known scenarios (spiral_tube, etc.), uses scripted trajectories
    that set ball positions programmatically for correct visualization.
    """
    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    data = mujoco.MjData(model)
    model_name = _get_model_name(mjcf_xml)

    # Apply damping override if specified
    if config.damping > 0:
        for i in range(model.njnt):
            model.jnt_damping[i] = config.damping

    # Set initial velocity
    if model.nq >= 7:  # Free joint: 7 qpos (pos + quat), 6 qvel
        data.qvel[0] = config.initial_vx
        data.qvel[1] = config.initial_vy
        data.qvel[2] = config.initial_vz

    trajectory = []
    step = 0
    impulse_applied = False
    scripted = model_name in ("spiral_tube",)

    while data.time < config.duration:
        if scripted and model_name == "spiral_tube":
            # Programmatically set ball position along the arc + tangent path
            x, y, z, vx, vy, vz = _spiral_tube_position(
                data.time, config.duration, speed=config.initial_vx or 2.0
            )
            if model.nq >= 7:
                data.qpos[0] = x
                data.qpos[1] = y
                data.qpos[2] = z
                data.qvel[0] = vx
                data.qvel[1] = vy
                data.qvel[2] = vz
            mujoco.mj_forward(model, data)
            data.time += model.opt.timestep
        else:
            # Standard MuJoCo physics
            if data.time >= config.force_start_time and not impulse_applied:
                data.qvel[2] += config.force_magnitude
                impulse_applied = True

            if model.nu > 0:
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
    """Run N simulations with different parameter sets."""
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


def generate_alternative_trajectory(
    config: SimConfig,
    initial_pos: tuple[float, float, float] | None = None,
    model_name: str = "",
) -> list[dict]:
    """Generate the alternative trajectory analytically.

    Dispatches to scenario-specific generators based on model_name.
    """
    if model_name == "spiral_tube":
        return _generate_spiral_tube_alternative(config)
    elif model_name == "frictionless_push":
        return _generate_frictionless_push_alternative(config, initial_pos)
    elif model_name == "galileo_drop":
        return _generate_galileo_drop_alternative(config)
    else:
        return _generate_disessa_alternative(config, initial_pos)


def _generate_disessa_alternative(
    config: SimConfig,
    initial_pos: tuple[float, float, float] | None = None,
) -> list[dict]:
    """DiSessa Ball: ball goes straight up (force replaces velocity)."""
    x0 = initial_pos[0] if initial_pos else 0.0
    z0 = initial_pos[2] if initial_pos else 0.5

    dt = config.duration / 200
    trajectory = []
    t = 0.0

    while t < config.duration:
        if t < config.force_start_time:
            x = x0 + config.initial_vx * t
            z = z0
            vx = config.initial_vx
            vz = 0.0
        else:
            x_at_force = x0 + config.initial_vx * config.force_start_time
            dt_after = t - config.force_start_time
            x = x_at_force
            vz_impulse = config.force_magnitude
            z = z0 + vz_impulse * dt_after
            vx = 0.0
            vz = vz_impulse

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


def _generate_spiral_tube_alternative(config: SimConfig) -> list[dict]:
    """Spiral Tube: ball continues curving after exit (curvilinear impetus).

    Arc phase: identical to Newtonian (ball is constrained inside tube).
    Exit phase: ball travels at constant speed but its heading (velocity
    direction) keeps rotating clockwise with exponentially decaying turn
    rate — the "dying away" of curvilinear impetus. This traces a spiral
    that gradually straightens into a line.
    Per McCloskey et al. (1980, 1983) and Freyd & Jones (1994).
    """
    duration = config.duration
    speed = config.initial_vx or 2.0
    arc_time = duration * 0.4
    turn_rate_0 = (math.pi / 2.0) / arc_time  # turning rate at exit (rad/s)
    decay_rate = 3.0  # controls how fast the curve straightens

    # Exit point (from _spiral_tube_position at t=arc_time)
    exit_x, _, exit_z, exit_vx, _, exit_vz = _spiral_tube_position(
        arc_time - 1e-9, duration, speed
    )
    # Exit velocity direction: heading angle measured from +x axis
    # At exit (top of arc), velocity is (+, 0) i.e. heading = 0
    exit_heading = math.atan2(exit_vz, exit_vx)
    exit_speed = math.sqrt(exit_vx**2 + exit_vz**2)

    dt = duration / 200
    trajectory = []
    t = 0.0

    # For exit phase: numerically integrate position from velocity
    alt_x = exit_x
    alt_z = exit_z
    prev_t = arc_time

    while t < duration:
        if t < arc_time:
            # Arc phase: identical to Newtonian (ball is inside tube)
            x, _, z, vx, _, vz = _spiral_tube_position(t, duration, speed)
        else:
            # After exit: constant speed, decaying turn rate
            dt_after = t - arc_time
            # Heading angle: integrate decaying turn rate
            # heading(t) = exit_heading - (turn_rate_0/decay_rate)*(1 - exp(-decay_rate*dt_after))
            heading = exit_heading - (turn_rate_0 / decay_rate) * (
                1 - math.exp(-decay_rate * dt_after)
            )
            vx = exit_speed * math.cos(heading)
            vz = exit_speed * math.sin(heading)

            # Integrate position (Euler step from previous point)
            step_dt = t - prev_t
            alt_x += vx * step_dt
            alt_z += vz * step_dt
            prev_t = t

            x = alt_x
            z = alt_z

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


def _generate_frictionless_push_alternative(
    config: SimConfig,
    initial_pos: tuple[float, float, float] | None = None,
) -> list[dict]:
    """Frictionless Push: ball slows and stops (dying away)."""
    x0 = initial_pos[0] if initial_pos else -3.0
    z0 = initial_pos[2] if initial_pos else 0.5

    dt = config.duration / 200
    trajectory = []
    t = 0.0
    vx = config.initial_vx
    x = x0
    damping = 3.0  # Chosen to show clear slowing

    while t < config.duration:
        # Ball decelerates due to "intrinsic damping" — alternative belief
        vx = vx * math.exp(-damping * dt)
        x = x + vx * dt

        trajectory.append({
            "t": round(t, 6),
            "x": round(x, 6),
            "y": 0.0,
            "z": round(z0, 6),
            "vx": round(vx, 6),
            "vy": 0.0,
            "vz": 0.0,
        })
        t += dt

    return trajectory


def _generate_galileo_drop_alternative(config: SimConfig) -> list[dict]:
    """Galileo's Drop: heavy ball falls faster (Ohm's p-prim).

    Tracks the heavy ball falling with enhanced gravity due to mass.
    """
    z0 = 5.0
    g = 9.81
    mass_factor = 2.0  # Heavy ball falls 2x faster in alternative belief

    dt = config.duration / 200
    trajectory = []
    t = 0.0
    vz = 0.0
    z = z0

    while t < config.duration:
        vz = vz - g * mass_factor * dt
        z = z + vz * dt

        trajectory.append({
            "t": round(t, 6),
            "x": round(-0.5, 6),  # Same x position as heavy ball
            "y": 0.0,
            "z": round(max(z, 0.0), 6),
            "vx": 0.0,
            "vy": 0.0,
            "vz": round(vz, 6),
        })
        t += dt

    return trajectory
