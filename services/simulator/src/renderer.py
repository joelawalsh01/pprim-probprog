"""Offscreen MuJoCo rendering using OSMesa."""

import base64
import io
import mujoco
import numpy as np
from PIL import Image

from .simulation import SimConfig


def render_frames(
    mjcf_xml: str,
    config: SimConfig,
    width: int = 640,
    height: int = 480,
    camera_name: str | None = "side",
    max_frames: int = 100,
) -> list[str]:
    """Render simulation frames as base64-encoded PNGs.

    Args:
        mjcf_xml: MJCF XML string.
        config: Simulation configuration.
        width: Frame width in pixels.
        height: Frame height in pixels.
        camera_name: Name of camera in MJCF, or None for free camera.
        max_frames: Maximum number of frames to return.

    Returns:
        List of base64-encoded PNG strings.
    """
    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    data = mujoco.MjData(model)

    # Apply damping override
    if config.damping > 0:
        for i in range(model.njnt):
            model.jnt_damping[i] = config.damping

    # Set initial velocity
    if model.nq >= 7:
        data.qvel[0] = config.initial_vx
        data.qvel[1] = config.initial_vy
        data.qvel[2] = config.initial_vz

    # Set up renderer
    renderer = mujoco.Renderer(model, height=height, width=width)

    total_steps = int(config.duration / model.opt.timestep)
    frame_interval = max(1, total_steps // max_frames)

    frames = []
    step = 0

    while data.time < config.duration:
        # Apply force
        if data.time >= config.force_start_time and model.nu > 0:
            data.ctrl[0] = config.force_magnitude
        elif model.nu > 0:
            data.ctrl[0] = 0.0

        mujoco.mj_step(model, data)
        step += 1

        if step % frame_interval == 0 and len(frames) < max_frames:
            renderer.update_scene(data, camera=camera_name)
            pixels = renderer.render()

            # Convert to base64 PNG
            img = Image.fromarray(pixels)
            buf = io.BytesIO()
            img.save(buf, format="PNG", optimize=True)
            b64 = base64.b64encode(buf.getvalue()).decode("ascii")
            frames.append(b64)

    renderer.close()
    return frames
