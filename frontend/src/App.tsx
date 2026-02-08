import React, { useState, useEffect, useCallback } from 'react';
import Dashboard from './components/Dashboard';
import type { SimulationResult, InferenceResult, Project } from './types';
import { getExample, getModels, simulate, runInference } from './api';

const DEFAULT_EXAMPLE = 'disessa_ball';
const STORAGE_KEY = 'disessa-balls-projects';
const ACTIVE_KEY = 'disessa-balls-active-project';

function generateId() {
  return Math.random().toString(36).slice(2, 10);
}

function loadProjects(): Project[] | null {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return null;
    return JSON.parse(raw);
  } catch {
    return null;
  }
}

function saveProjects(projects: Project[]) {
  // Strip non-serializable/large data (frames) to keep localStorage small
  const slim = projects.map(p => ({
    ...p,
    simResult: p.simResult ? { ...p.simResult, frames: undefined } : null,
  }));
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(slim));
  } catch {
    // localStorage full — silently ignore
  }
}

function loadActiveId(): string | null {
  try {
    return localStorage.getItem(ACTIVE_KEY);
  } catch {
    return null;
  }
}

function saveActiveId(id: string) {
  try {
    localStorage.setItem(ACTIVE_KEY, id);
  } catch {}
}

export default function App() {
  const [projects, setProjects] = useState<Project[]>([]);
  const [activeProjectId, setActiveProjectId] = useState<string>('');
  const [simLoading, setSimLoading] = useState(false);
  const [inferLoading, setInferLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [defaultPyroCode, setDefaultPyroCode] = useState(FALLBACK_PYRO);
  const [initialized, setInitialized] = useState(false);

  // Load from localStorage or fetch defaults on mount
  useEffect(() => {
    (async () => {
      // Try to load saved projects
      const saved = loadProjects();
      const savedActiveId = loadActiveId();

      // Fetch example data from API
      let exampleMjcf = FALLBACK_MJCF;
      let examplePyro = FALLBACK_PYRO;
      try {
        const [exampleRes, modelsRes] = await Promise.all([
          getExample(DEFAULT_EXAMPLE),
          getModels(),
        ]);
        exampleMjcf = exampleRes.mjcf_xml;
        if (modelsRes.models.length > 0) {
          examplePyro = modelsRes.models[0].code;
        }
      } catch {
        // Services might not be up yet — use fallbacks
      }
      setDefaultPyroCode(examplePyro);

      if (saved && saved.length > 0) {
        // Update the DiSessa Ball project with fresh API data if it exists
        const updated = saved.map(p => {
          if (p.name === 'DiSessa Ball' && exampleMjcf !== FALLBACK_MJCF) {
            return { ...p, mjcfXml: exampleMjcf, pyroCode: examplePyro };
          }
          return p;
        });
        setProjects(updated);
        setActiveProjectId(
          savedActiveId && updated.some(p => p.id === savedActiveId)
            ? savedActiveId
            : updated[0].id
        );
      } else {
        // First visit — create the default DiSessa Ball project
        const defaultProject: Project = {
          id: generateId(),
          name: 'DiSessa Ball',
          mjcfXml: exampleMjcf,
          pyroCode: examplePyro,
          simResult: null,
          inferResult: null,
        };
        setProjects([defaultProject]);
        setActiveProjectId(defaultProject.id);
      }
      setInitialized(true);
    })();
  }, []);

  // Persist projects to localStorage whenever they change
  useEffect(() => {
    if (initialized && projects.length > 0) {
      saveProjects(projects);
    }
  }, [projects, initialized]);

  // Persist active project ID
  useEffect(() => {
    if (activeProjectId) {
      saveActiveId(activeProjectId);
    }
  }, [activeProjectId]);

  const activeProject = projects.find(p => p.id === activeProjectId) || projects[0];

  const updateActiveProject = useCallback((updates: Partial<Project>) => {
    setProjects(prev => prev.map(p =>
      p.id === activeProjectId ? { ...p, ...updates } : p
    ));
  }, [activeProjectId]);

  const handleMjcfChange = useCallback((xml: string) => {
    updateActiveProject({ mjcfXml: xml });
  }, [updateActiveProject]);

  const handlePyroCodeChange = useCallback((code: string) => {
    updateActiveProject({ pyroCode: code });
  }, [updateActiveProject]);

  const handleSimulate = useCallback(async () => {
    if (!activeProject) return;
    setSimLoading(true);
    setError(null);
    try {
      const result = await simulate({
        mjcf_xml: activeProject.mjcfXml,
        render_frames: true,
        include_naive: true,
        max_frames: 60,
      });
      updateActiveProject({ simResult: result });
    } catch (e: any) {
      setError(`Simulation error: ${e.message}`);
    } finally {
      setSimLoading(false);
    }
  }, [activeProject, updateActiveProject]);

  const handleInfer = useCallback(async () => {
    if (!activeProject) return;
    setInferLoading(true);
    setError(null);
    try {
      const result = await runInference({
        trajectory_type: 'naive',
        method: 'svi',
        num_steps: 500,
        model_code: activeProject.pyroCode,
      });
      updateActiveProject({ inferResult: result });
    } catch (e: any) {
      setError(`Inference error: ${e.message}`);
    } finally {
      setInferLoading(false);
    }
  }, [activeProject, updateActiveProject]);

  const handleAddProject = useCallback(() => {
    const newProject: Project = {
      id: generateId(),
      name: 'New Project',
      mjcfXml: TEMPLATE_MJCF,
      pyroCode: defaultPyroCode,
      simResult: null,
      inferResult: null,
    };
    setProjects(prev => [...prev, newProject]);
    setActiveProjectId(newProject.id);
  }, [defaultPyroCode]);

  const handleCloseProject = useCallback((id: string) => {
    setProjects(prev => {
      if (prev.length <= 1) return prev; // Can't close last project
      const next = prev.filter(p => p.id !== id);
      // If we closed the active project, switch to the first remaining
      if (id === activeProjectId) {
        setActiveProjectId(next[0].id);
      }
      return next;
    });
  }, [activeProjectId]);

  const handleSwitchProject = useCallback((id: string) => {
    setActiveProjectId(id);
    setError(null);
  }, []);

  const handleRenameProject = useCallback((id: string, name: string) => {
    setProjects(prev => prev.map(p =>
      p.id === id ? { ...p, name } : p
    ));
  }, []);

  if (!initialized || !activeProject) {
    return (
      <div style={{ background: '#1a1b26', height: '100vh', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
        <div style={{ color: '#565f89', fontSize: '14px' }}>Loading...</div>
      </div>
    );
  }

  return (
    <Dashboard
      projects={projects}
      activeProjectId={activeProjectId}
      onSwitchProject={handleSwitchProject}
      onAddProject={handleAddProject}
      onCloseProject={handleCloseProject}
      onRenameProject={handleRenameProject}
      mjcfXml={activeProject.mjcfXml}
      onMjcfChange={handleMjcfChange}
      pyroCode={activeProject.pyroCode}
      onPyroCodeChange={handlePyroCodeChange}
      simResult={activeProject.simResult}
      inferResult={activeProject.inferResult}
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

// Template for new projects
const TEMPLATE_MJCF = `<mujoco model="new_model">
  <option timestep="0.002" gravity="0 0 0" integrator="RK4"/>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>

    <body name="ball" pos="0 0 0.5">
      <joint type="free" damping="0"/>
      <geom type="sphere" size="0.08" mass="1.0" rgba="0.2 0.6 1.0 1"
            contype="0" conaffinity="0"/>
      <site name="force_point" pos="0 0 0" size="0.01"/>
    </body>

    <camera name="side" pos="0 -4 1.5" xyaxes="1 0 0 0 0.3 1"/>
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
