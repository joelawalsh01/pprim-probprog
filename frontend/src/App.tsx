import React, { useState, useEffect, useCallback } from 'react';
import Dashboard from './components/Dashboard';
import type { SimulationResult, InferenceResult } from './types';
import { getExample, getModels, simulate, runInference } from './api';

const DEFAULT_EXAMPLE = 'disessa_ball';

export default function App() {
  const [mjcfXml, setMjcfXml] = useState('');
  const [pyroCode, setPyroCode] = useState('');
  const [simResult, setSimResult] = useState<SimulationResult | null>(null);
  const [inferResult, setInferResult] = useState<InferenceResult | null>(null);
  const [simLoading, setSimLoading] = useState(false);
  const [inferLoading, setInferLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Load default example on mount
  useEffect(() => {
    (async () => {
      try {
        const [exampleRes, modelsRes] = await Promise.all([
          getExample(DEFAULT_EXAMPLE),
          getModels(),
        ]);
        setMjcfXml(exampleRes.mjcf_xml);
        if (modelsRes.models.length > 0) {
          setPyroCode(modelsRes.models[0].code);
        }
      } catch {
        // Services might not be up yet — use embedded defaults
        setMjcfXml(FALLBACK_MJCF);
        setPyroCode(FALLBACK_PYRO);
      }
    })();
  }, []);

  const handleSimulate = useCallback(async () => {
    setSimLoading(true);
    setError(null);
    try {
      const result = await simulate({
        mjcf_xml: mjcfXml,
        render_frames: true,
        include_naive: true,
        max_frames: 60,
      });
      setSimResult(result);
    } catch (e: any) {
      setError(`Simulation error: ${e.message}`);
    } finally {
      setSimLoading(false);
    }
  }, [mjcfXml]);

  const handleInfer = useCallback(async () => {
    setInferLoading(true);
    setError(null);
    try {
      const result = await runInference({
        trajectory_type: 'naive',
        method: 'svi',
        num_steps: 500,
        model_code: pyroCode,
      });
      setInferResult(result);
    } catch (e: any) {
      setError(`Inference error: ${e.message}`);
    } finally {
      setInferLoading(false);
    }
  }, [pyroCode]);

  return (
    <Dashboard
      mjcfXml={mjcfXml}
      onMjcfChange={setMjcfXml}
      pyroCode={pyroCode}
      onPyroCodeChange={setPyroCode}
      simResult={simResult}
      inferResult={inferResult}
      onSimulate={handleSimulate}
      onInfer={handleInfer}
      simLoading={simLoading}
      inferLoading={inferLoading}
      error={error}
    />
  );
}

// Fallback content when services aren't available
const FALLBACK_MJCF = `<mujoco model="disessa_ball">
  <option timestep="0.002" gravity="0 0 0" integrator="RK4"/>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.15 0.15 0.2" rgb2="0.2 0.2 0.25"/>
    <material name="ground" texture="grid" texrepeat="8 8"
              specular="0.3" shininess="0.3"/>
  </asset>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1" diffuse="0.8 0.8 0.8" castshadow="false"/>
    <light pos="0 -3 3" dir="0 0.5 -0.5" diffuse="0.4 0.4 0.5"/>

    <geom type="plane" size="5 5 0.01" material="ground" contype="0" conaffinity="0"/>

    <body name="ball" pos="-2 0 0.5">
      <joint type="free" damping="0"/>
      <geom type="sphere" size="0.08" mass="1.0" rgba="0.2 0.6 1.0 1"
            contype="0" conaffinity="0"/>
      <site name="force_point" pos="0 0 0" size="0.01"/>
    </body>

    <camera name="side" pos="0 -4 1.5" xyaxes="1 0 0 0 0.3 1"/>
    <camera name="top" pos="0 0 5" xyaxes="1 0 0 0 1 0"/>
  </worldbody>

  <actuator>
    <motor name="upward_force" site="force_point" gear="0 0 1 0 0 0"
           ctrllimited="true" ctrlrange="0 50"/>
  </actuator>

  <sensor>
    <framepos name="ball_pos" objtype="body" objname="ball"/>
    <framelinvel name="ball_vel" objtype="body" objname="ball"/>
  </sensor>
</mujoco>`;

const FALLBACK_PYRO = `import torch
import pyro
import pyro.distributions as dist

def physics_model(observed_trajectory, dt=0.01):
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
    return positions`;
