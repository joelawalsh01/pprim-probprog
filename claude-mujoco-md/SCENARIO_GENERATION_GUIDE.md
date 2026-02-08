# Scenario Generation Guide

A complete reference for generating physics misconception scenarios for the **pprim-probprog** dashboard. Paste this into a Claude conversation (or any LLM) to produce all three artifacts needed for a new scenario: MuJoCo XML, Pyro model code, and P-Prim config JSON.

---

## Overview

The dashboard has three editable panels. For each new scenario, the LLM should generate:

1. **MuJoCo MJCF XML** — the physics scene (objects, forces, sensors, cameras)
2. **Pyro Model (Python)** — the probabilistic model with learnable cognitive physics parameters
3. **P-Prim Config (JSON)** — maps model parameters to p-prim interpretations

Each output is pasted into the corresponding panel in the dashboard. The researcher then clicks **Simulate** → **Run Inference** to get meaningful interpretation.

---

## Prompt Flow

When a user asks you to generate a scenario, follow these steps:

### Step 1: Understand the scenario
Ask (or accept) a description of the physics scenario:
- What objects are involved?
- What forces act on them?
- What is the initial state and duration?

### Step 2: Identify alternative conceptions
Ask about known p-prims or alternative conceptions:
- What do students commonly predict for this scenario?
- What is the Newtonian (correct) prediction?
- What physical principle does the alternative conception violate?

### Step 3: Design the parameter mapping
For each alternative conception, identify:
- Which physical parameter would need to differ from its Newtonian value to produce the alternative prediction?
- What range of values corresponds to the alternative vs. Newtonian behavior?
- What prior distribution is appropriate (centered between the two)?

### Step 4: Output three code blocks
Produce clearly labeled outputs:
- ` ```xml ` — MuJoCo MJCF XML → paste into the **MuJoCo Code** panel
- ` ```python ` — Pyro Model → paste into the **Pyro Model > Code** tab
- ` ```json ` — P-Prim Config → paste into the **Pyro Model > P-Prims** tab (via "Paste JSON")

---

## Part 1: MJCF XML Generation

### Structure

Every MuJoCo scene is an XML document with `<mujoco>` as the root element:

```xml
<mujoco model="scene_name">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <asset>
    <!-- Materials, textures, meshes -->
  </asset>

  <worldbody>
    <!-- Static and dynamic bodies -->
  </worldbody>

  <actuator>
    <!-- Motors, forces, torques -->
  </actuator>

  <sensor>
    <!-- Position, velocity, force sensors -->
  </sensor>
</mujoco>
```

### Key Top-Level Elements

| Element | Purpose |
|---------|---------|
| `<option>` | Simulation parameters: timestep, gravity, integrator |
| `<default>` | Default attribute values for geoms, joints, etc. |
| `<asset>` | Textures, materials, meshes, heightfields |
| `<worldbody>` | Scene graph: bodies, geoms, joints, sites |
| `<actuator>` | Force/torque generators attached to joints or bodies |
| `<sensor>` | Measurements: positions, velocities, forces |

### Physical Primitives

#### Bodies
```xml
<body name="ball" pos="0 0 1">
  <joint type="free"/>  <!-- 6-DOF -->
  <geom type="sphere" size="0.05" mass="1.0" rgba="0.2 0.6 1 1"/>
</body>
```

#### Geom Types
| Type | Size Parameters | Example |
|------|----------------|---------|
| `sphere` | `radius` | `size="0.05"` |
| `box` | `half-x half-y half-z` | `size="0.1 0.1 0.1"` |
| `capsule` | `radius half-length` | `size="0.02 0.1"` |
| `cylinder` | `radius half-length` | `size="0.05 0.1"` |
| `plane` | `x-half y-half spacing` | `size="5 5 0.1"` |
| `ellipsoid` | `x-radius y-radius z-radius` | `size="0.1 0.05 0.05"` |

#### Joint Types
| Type | DOF | Use |
|------|-----|-----|
| `free` | 6 | Unconstrained body (projectile, floating object) |
| `hinge` | 1 | Rotation around axis |
| `slide` | 1 | Translation along axis |
| `ball` | 3 | Spherical joint (3 rotational DOF) |

#### Actuators
```xml
<actuator>
  <!-- Force applied to a joint -->
  <motor name="push" joint="slider" gear="1" ctrllimited="true" ctrlrange="-10 10"/>

  <!-- Force applied at a site (body point) -->
  <motor name="thrust" site="force_point" gear="0 0 1 0 0 0"/>
  <!-- gear="fx fy fz tx ty tz" for site actuators -->
</actuator>
```

#### Sensors
```xml
<sensor>
  <framepos name="ball_pos" objtype="body" objname="ball"/>
  <framevelocity name="ball_vel" objtype="body" objname="ball"/>
</sensor>
```

### Design Guidelines

- **Set gravity appropriately** — turn it off (`gravity="0 0 0"`) when it's not part of the question being studied
- **Use `contype="0" conaffinity="0"`** on geoms that shouldn't collide (e.g. visual-only ground planes)
- **Include cameras** — at least a side view for the animation panel
- **Include sensors** — the simulator needs `framepos` to extract trajectories
- **Name everything** — bodies, joints, actuators, sensors all need names
- **Use SI units** — meters, kilograms, seconds, Newtons

### Checklist

When a scenario description is underspecified, ask about:

- [ ] Object shape, size, mass, color
- [ ] Starting position and velocity
- [ ] What forces are applied, when, in what direction
- [ ] Friction, damping, restitution (if relevant)
- [ ] Duration, timestep, gravity
- [ ] Camera angle

---

## Part 2: Pyro Model Generation

The Pyro model is the probabilistic program that the dashboard uses for Bayesian inference. It defines:
1. **Prior distributions** over cognitive physics parameters
2. **A forward simulation** that produces a predicted trajectory from those parameters
3. **A likelihood** that scores how well the prediction matches an observed trajectory

### Structure Template

```python
import torch
import pyro
import pyro.distributions as dist

def physics_model(observed_trajectory, dt=0.01):
    n_steps = observed_trajectory.shape[0]

    # 1. PRIORS — sample cognitive physics parameters
    param_name = pyro.sample("param_name", dist.SomeDistribution(...))
    sigma = pyro.sample("sigma", dist.LogNormal(-2.0, 1.0))

    # 2. FORWARD SIMULATION — generate predicted trajectory
    positions = torch.zeros(n_steps, 2)  # x, z
    # ... physics loop using the sampled parameters ...

    # 3. LIKELIHOOD — score prediction against observation
    pyro.sample(
        "obs",
        dist.Normal(positions, sigma).to_event(2),
        obs=observed_trajectory,
    )
    return positions
```

### Designing Cognitive Physics Parameters

The key insight: **each parameter should interpolate between an alternative conception and Newtonian behavior**.

1. **Identify the physical principle** the alternative conception violates
2. **Create a parameter** whose value at one extreme reproduces the alternative prediction, and at the other extreme reproduces the Newtonian prediction
3. **Choose a prior** centered between the extremes (agnostic starting point)

#### Common Patterns

| P-prim | Physical principle violated | Parameter | Alternative value | Newtonian value | Good prior |
|--------|---------------------------|-----------|-------------------|-----------------|------------|
| Force as mover | Superposition of forces | `velocity_persistence` | 0 (force replaces velocity) | 1 (force adds) | `Beta(2, 2)` on [0,1] |
| Dying away | Newton's 1st law (inertia) | `lateral_damping` | High (>2) | ~0 | `LogNormal(0, 1)` |
| Heavier falls faster | Independence of mass and fall rate | `mass_gravity_coupling` | 1 (mass affects fall speed) | 0 (independent) | `Beta(2, 2)` |
| Bigger means slower | F=ma misapplication | `mass_speed_coupling` | High | 0 | `LogNormal(0, 1)` |
| Gravity as switch | Gravity always acts | `gravity_onset` | Delayed | 0 (always on) | `Exponential(1)` |

#### Prior Selection Guide

| Distribution | Range | Use when |
|-------------|-------|----------|
| `Beta(2, 2)` | [0, 1] | Parameter naturally bounded, center at 0.5 |
| `Beta(a, b)` | [0, 1] | Bounded, asymmetric prior (higher a → right-skewed) |
| `LogNormal(mu, sigma)` | (0, ∞) | Positive parameter, log-scale uncertainty |
| `Normal(mu, sigma)` | (-∞, ∞) | Unbounded parameter (rare in physics) |
| `Exponential(rate)` | [0, ∞) | Positive, expect small values |

### Forward Simulation Guidelines

- Use the **same coordinate system** as the MuJoCo scene (x = horizontal, z = vertical)
- The initial conditions (position, velocity) should match the MuJoCo simulation
- The force timing and direction should match the MuJoCo actuator setup
- Always include `sigma` (observation noise) — without it, inference may fail to converge

### Important: Function Signature

The model **must** have the signature:
```python
def physics_model(observed_trajectory, dt=0.01):
```
The dashboard's inference engine calls it with this exact API.

---

## Part 3: P-Prim Config Generation

The P-Prim config is a JSON structure that tells the dashboard how to interpret posterior parameter values in terms of alternative conceptions and p-prims.

### JSON Schema

```json
{
  "version": 1,
  "summaryTemplate": null,
  "mappings": [
    {
      "parameter": "the_pyro_sample_name",
      "pprimName": "Human-readable p-prim label",
      "description": "What this parameter means in the model and how it connects to the p-prim",
      "priorDist": "Beta(2,2)",
      "priorMean": 0.5,
      "range": [0, 1],
      "color": "#7aa2f7",
      "thresholds": [
        {
          "max": 0.3,
          "label": "Description of what low values mean",
          "conception": "alternative"
        },
        {
          "min": 0.3,
          "max": 0.7,
          "label": "Description of what middle values mean",
          "conception": "mixed"
        },
        {
          "min": 0.7,
          "label": "Description of what high values mean",
          "conception": "newtonian"
        }
      ],
      "lowConclusion": "Text shown when posterior mean is below prior mean",
      "highConclusion": "Text shown when posterior mean is above prior mean"
    }
  ]
}
```

### Field Reference

| Field | Type | Description |
|-------|------|-------------|
| `parameter` | string | Must match the `pyro.sample("name", ...)` in the Pyro model exactly |
| `pprimName` | string | Human-readable label (e.g. "Force as mover") |
| `description` | string | Detailed explanation shown in the reasoning panel |
| `priorDist` | string | Display string for the prior (e.g. "Beta(2,2)") |
| `priorMean` | number | Expected value of the prior distribution |
| `range` | [min, max] | Display range for histograms |
| `color` | string (hex) | Color used in the Prior/Posterior panel |
| `thresholds` | array | Interpretation bands (see below) |
| `lowConclusion` | string | Conclusion text when posterior < prior |
| `highConclusion` | string | Conclusion text when posterior > prior |

### Thresholds

Each threshold defines a band of posterior values and its interpretation:

| Field | Type | Description |
|-------|------|-------------|
| `min` | number or null | Lower bound (null = no lower bound) |
| `max` | number or null | Upper bound (null = no upper bound) |
| `label` | string | What this band means |
| `conception` | string | One of: `"alternative"`, `"newtonian"`, `"mixed"`, `"neutral"` |

The `conception` field drives the summary: the dashboard counts how many parameters fall into "alternative" vs "newtonian" bands and generates an overall summary.

### Guidelines

- **Every `pyro.sample` parameter** (except `sigma`/`obs`) should have a mapping
- Parameters without thresholds (like `force_magnitude`, `mass`) can still have mappings — they'll show in the reasoning panel with just conclusion text
- `sigma` (observation noise) can optionally have a mapping, but it's not diagnostic of p-prims
- The `conception` field on thresholds determines the overall summary verdict
- Choose colors that are visually distinct: `#7aa2f7` (blue), `#f7768e` (red), `#9ece6a` (green), `#e0af68` (yellow), `#bb9af7` (purple), `#73daca` (teal), `#ff9e64` (orange)

---

## Part 4: Worked Example — DiSessa Ball

### Scenario

A ball moves horizontally to the right at 3 m/s. At t=0.5s, a constant upward force of 10N is applied. What path does the ball follow?

- **Alternative prediction**: Ball goes straight up (force "replaces" velocity)
- **Newtonian prediction**: Ball curves diagonally up-right (force "adds to" velocity)

**Known p-prims:**
- *Force as mover* — force sets velocity directly → `velocity_persistence` near 0
- *Dying away* — motion naturally dissipates → `lateral_damping` high

### Output 1: MuJoCo MJCF XML

Paste into the **MuJoCo Code** panel:

```xml
<mujoco model="disessa_ball">
  <option timestep="0.002" gravity="0 0 0" integrator="RK4"/>
  <!-- No gravity: isolate the force-vs-velocity question -->

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.15 0.15 0.2" rgb2="0.2 0.2 0.25"/>
    <material name="ground" texture="grid" texrepeat="8 8"
              specular="0.3" shininess="0.3"/>
  </asset>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1" diffuse="0.8 0.8 0.8" castshadow="false"/>
    <light pos="0 -3 3" dir="0 0.5 -0.5" diffuse="0.4 0.4 0.5"/>

    <!-- Ground reference plane (visual only, no collision) -->
    <geom type="plane" size="5 5 0.01" material="ground" contype="0" conaffinity="0"/>

    <!-- The ball -->
    <body name="ball" pos="-2 0 0.5">
      <joint type="free" damping="0"/>
      <geom type="sphere" size="0.08" mass="1.0" rgba="0.2 0.6 1.0 1"
            contype="0" conaffinity="0"/>
      <site name="force_point" pos="0 0 0" size="0.01"/>
    </body>

    <!-- Cameras -->
    <camera name="side" pos="0 -4 1.5" xyaxes="1 0 0 0 0.3 1"/>
    <camera name="top" pos="0 0 5" xyaxes="1 0 0 0 1 0"/>
  </worldbody>

  <actuator>
    <!-- Upward force applied at ball center -->
    <motor name="upward_force" site="force_point" gear="0 0 1 0 0 0"
           ctrllimited="true" ctrlrange="0 50"/>
  </actuator>

  <sensor>
    <framepos name="ball_pos" objtype="body" objname="ball"/>
    <framelinvel name="ball_vel" objtype="body" objname="ball"/>
  </sensor>
</mujoco>
```

### Output 2: Pyro Model

Paste into the **Pyro Model > Code** tab:

```python
import torch
import pyro
import pyro.distributions as dist

def physics_model(observed_trajectory, dt=0.01):
    """Probabilistic model of naive physics beliefs.

    Parameters being inferred:
    - velocity_persistence: 0 = force replaces velocity (naive), 1 = force adds (Newtonian)
    - lateral_damping: how quickly horizontal velocity decays (high = naive belief)
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
```

### Output 3: P-Prim Config JSON

Paste into the **Pyro Model > P-Prims** tab (click "Paste JSON"):

```json
{
  "version": 1,
  "summaryTemplate": null,
  "mappings": [
    {
      "parameter": "velocity_persistence",
      "pprimName": "Force as mover",
      "description": "Controls what happens to existing velocity when a new force is applied. At 0, the force completely replaces the current velocity (the ball forgets it was moving horizontally). At 1, the force adds to the current velocity (Newtonian superposition).",
      "priorDist": "Beta(2,2)",
      "priorMean": 0.5,
      "range": [0, 1],
      "color": "#7aa2f7",
      "thresholds": [
        { "max": 0.3, "label": "Force replaces velocity (alternative conception / 'force as mover' p-prim)", "conception": "alternative" },
        { "min": 0.3, "max": 0.7, "label": "Mixed beliefs — partial persistence", "conception": "mixed" },
        { "min": 0.7, "label": "Force adds to velocity (Newtonian / 'force as deflector')", "conception": "newtonian" }
      ],
      "lowConclusion": "Strong evidence for the 'force as mover' p-prim — the person's predicted trajectory is best explained by a model where force replaces existing velocity.",
      "highConclusion": "Beliefs broadly consistent with Newtonian mechanics — force adds to existing motion."
    },
    {
      "parameter": "lateral_damping",
      "pprimName": "Dying away",
      "description": "Controls how quickly horizontal velocity decays over time. High values mean horizontal motion dies quickly — like an implicit belief that objects naturally stop moving (Aristotelian). Low values mean motion persists (Newton's first law).",
      "priorDist": "LogNormal(0,1)",
      "priorMean": 1.65,
      "range": [0, 10],
      "color": "#f7768e",
      "thresholds": [
        { "max": 0.5, "label": "Low damping — motion persists (Newtonian)", "conception": "newtonian" },
        { "min": 0.5, "max": 2.0, "label": "Moderate damping", "conception": "mixed" },
        { "min": 2.0, "label": "High damping — motion dies quickly (Aristotelian)", "conception": "alternative" }
      ],
      "lowConclusion": "Consistent with Newton's first law — objects in motion stay in motion.",
      "highConclusion": "The person expects horizontal motion to dissipate, as if objects naturally come to rest unless actively pushed."
    },
    {
      "parameter": "force_magnitude",
      "pprimName": "Force strength",
      "description": "The perceived strength of the applied force. Interacts with mass to determine vertical acceleration (a = F/m).",
      "priorDist": "LogNormal(1,0.5)",
      "priorMean": 3.08,
      "range": [0, 20],
      "color": "#9ece6a",
      "thresholds": [],
      "lowConclusion": "The model found a low force magnitude best reproduces the observed vertical motion.",
      "highConclusion": "The model found a high force magnitude best reproduces the observed vertical motion."
    },
    {
      "parameter": "mass",
      "pprimName": "Perceived mass",
      "description": "The perceived mass of the object. Together with force_magnitude, determines vertical acceleration. Mass and force are partially degenerate (only their ratio matters).",
      "priorDist": "LogNormal(0,0.5)",
      "priorMean": 1.13,
      "range": [0, 5],
      "color": "#e0af68",
      "thresholds": [],
      "lowConclusion": "The inferred mass is low, implying higher perceived acceleration.",
      "highConclusion": "The inferred mass is high, implying lower perceived acceleration."
    },
    {
      "parameter": "sigma",
      "pprimName": "Observation noise",
      "description": "The observation noise — how tightly the model must match the observed trajectory.",
      "priorDist": "LogNormal(-2,1)",
      "priorMean": 0.2,
      "range": [0, 1],
      "color": "#bb9af7",
      "thresholds": [],
      "lowConclusion": "Low noise — the model fits the trajectory tightly.",
      "highConclusion": "High noise — the model has a loose fit to the trajectory."
    }
  ]
}
```

---

## Part 5: Common P-Prim Mappings Reference

Use this table to quickly identify parameter designs for common alternative conceptions:

| P-prim | Description | Model parameter | Alternative value | Newtonian value | Prior |
|--------|-------------|----------------|-------------------|-----------------|-------|
| **Force as mover** | Force sets velocity directly, replacing existing motion | `velocity_persistence` | Near 0 | Near 1 | `Beta(2,2)` |
| **Dying away** | Motion naturally dissipates without sustained force | `lateral_damping` | High (>2) | Near 0 | `LogNormal(0,1)` |
| **Bigger means slower** | Heavier objects move more slowly under same force | `mass_speed_coupling` | High | 0 | `LogNormal(0,1)` |
| **Closer means stronger** | Force effect depends on proximity | `force_distance_decay` | High | 0 | `LogNormal(0,1)` |
| **Continuing push** | Objects keep accelerating as long as force is "in them" | `force_persistence` | Extended | Impulse only | `LogNormal(0,1)` |
| **Gravity as switch** | Gravity only acts when object "should" fall | `gravity_delay` | Positive | 0 | `Exponential(1)` |
| **Straight then curve** | Object moves straight first, then curves | `velocity_persistence` | Time-varying | Constant ~1 | `Beta(2,2)` |
| **Heavier falls faster** | Mass affects fall rate in vacuum | `mass_gravity_coupling` | 1 (coupled) | 0 (independent) | `Beta(2,2)` |

For p-prims not in this table, reason about what parameter(s) would need to differ from Newtonian values to reproduce the alternative prediction, and design the mapping accordingly.

---

## Output Format Summary

When the user describes a scenario, always output **three clearly labeled code blocks**:

1. **MuJoCo MJCF XML** (` ```xml `)
   - Complete, valid XML
   - Comments explaining non-obvious choices
   - Cameras and sensors included
   - SI units

2. **Pyro Model** (` ```python `)
   - Function signature: `def physics_model(observed_trajectory, dt=0.01):`
   - Priors as `pyro.sample(...)` calls
   - Forward simulation loop matching the MuJoCo scene
   - `sigma` for observation noise
   - `pyro.sample("obs", ...)` likelihood at the end

3. **P-Prim Config** (` ```json `)
   - One mapping per diagnostic parameter
   - Thresholds with `conception` tags
   - Conclusion text for low/high values
   - Colors for visualization

Tell the researcher which panel each output goes into.
