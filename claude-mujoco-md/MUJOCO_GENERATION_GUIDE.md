# MuJoCo MJCF XML Generation Guide

A comprehensive reference for generating MuJoCo simulation scenes. Paste this into a Claude conversation to get well-structured MJCF XML for physics scenarios.

---

## 1. MJCF XML Structure

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
| `<option>` | Simulation parameters: timestep, gravity, integrator, solver |
| `<default>` | Default attribute values for geoms, joints, etc. |
| `<asset>` | Textures, materials, meshes, heightfields |
| `<worldbody>` | Scene graph: bodies, geoms, joints, sites |
| `<actuator>` | Force/torque generators attached to joints or bodies |
| `<sensor>` | Measurements: positions, velocities, forces |
| `<keyframe>` | Saved states (initial conditions) |

---

## 2. Physical Primitives

### Bodies
Bodies are the basic dynamic elements. They can contain geoms (shapes), joints (degrees of freedom), and sites (attachment points).

```xml
<body name="ball" pos="0 0 1">
  <joint type="free"/>  <!-- 6-DOF: can translate and rotate freely -->
  <geom type="sphere" size="0.05" mass="1.0" rgba="0.2 0.6 1 1"/>
</body>
```

### Geom Types
| Type | Size Parameters | Example |
|------|----------------|---------|
| `sphere` | `radius` | `size="0.05"` |
| `box` | `half-x half-y half-z` | `size="0.1 0.1 0.1"` |
| `capsule` | `radius half-length` | `size="0.02 0.1"` |
| `cylinder` | `radius half-length` | `size="0.05 0.1"` |
| `plane` | `x-half y-half spacing` | `size="5 5 0.1"` (infinite if 0 0) |
| `ellipsoid` | `x-radius y-radius z-radius` | `size="0.1 0.05 0.05"` |
| `mesh` | reference to asset | `mesh="obj_name"` |

### Joint Types
| Type | DOF | Use |
|------|-----|-----|
| `free` | 6 | Unconstrained body (projectile, floating object) |
| `hinge` | 1 | Rotation around axis |
| `slide` | 1 | Translation along axis |
| `ball` | 3 | Spherical joint (3 rotational DOF) |

```xml
<!-- Hinge joint with limits -->
<joint name="elbow" type="hinge" axis="0 1 0" range="-90 90" damping="0.1"/>

<!-- Slide joint -->
<joint name="piston" type="slide" axis="0 0 1" range="0 0.5"/>
```

### Actuators

```xml
<actuator>
  <!-- Force applied to a joint -->
  <motor name="push" joint="slider" gear="1" ctrllimited="true" ctrlrange="-10 10"/>

  <!-- Force applied at a site (body point) -->
  <motor name="thrust" site="force_point" gear="0 0 1 0 0 0"/>
  <!-- gear="fx fy fz tx ty tz" for site actuators -->
</actuator>
```

### Sensors

```xml
<sensor>
  <framepos name="ball_pos" objtype="body" objname="ball"/>
  <framevelocity name="ball_vel" objtype="body" objname="ball"/>
  <actuatorfrc name="force_sensor" actuator="thrust"/>
</sensor>
```

---

## 3. Material Properties

### Friction
MuJoCo uses 3-parameter friction: `friction="slide spin roll"`
- Default: `friction="1 0.005 0.0001"`
- Ice/slippery: `friction="0.01 0.001 0.00001"`
- Rubber: `friction="2 0.01 0.001"`

### Density & Mass
Specify either `mass` (direct) or `density` (computed from volume):
```xml
<geom type="sphere" size="0.05" mass="1.0"/>     <!-- Direct mass -->
<geom type="sphere" size="0.05" density="1000"/>  <!-- Water density -->
```

### Damping
Applied to joints to simulate friction/drag:
```xml
<joint type="free" damping="0.5"/>  <!-- Linear + angular damping -->
```

### Contact Properties
```xml
<default>
  <geom condim="3" conaffinity="1" contype="1" solref="0.01 1" solimp="0.9 0.95 0.001"/>
</default>
```

- `solref`: Contact solver reference (timeconst, dampratio)
- `solimp`: Solver impedance (dmin, dmax, width)
- `condim`: Contact dimensionality (1=frictionless, 3=friction, 6=elliptic)

---

## 4. Common Scene Patterns

### Projectile Motion
```xml
<mujoco model="projectile">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>
    <geom type="plane" size="10 10 0.1" rgba="0.3 0.3 0.3 1"/>

    <body name="ball" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="sphere" size="0.05" mass="0.1" rgba="1 0.3 0.3 1"/>
    </body>
  </worldbody>
</mujoco>
```

Set initial velocity via `mj_data.qvel`:
```python
data.qvel[0] = 5.0   # vx
data.qvel[2] = 10.0  # vz (up)
```

### Applied Force at a Point
Use a site + site actuator:
```xml
<body name="ball" pos="0 0 0.5">
  <joint type="free"/>
  <geom type="sphere" size="0.05" mass="1.0" rgba="0.2 0.6 1 1"/>
  <site name="force_point" pos="0 0 0" size="0.01"/>
</body>

<actuator>
  <motor name="upward_force" site="force_point" gear="0 0 1 0 0 0"/>
</actuator>
```

### Ground Plane with Grid Texture
```xml
<asset>
  <texture name="grid" type="2d" builtin="checker" width="512" height="512"
           rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3"/>
  <material name="grid_mat" texture="grid" texrepeat="10 10"/>
</asset>

<worldbody>
  <geom type="plane" size="10 10 0.1" material="grid_mat"/>
</worldbody>
```

### Camera Setup
```xml
<worldbody>
  <!-- Fixed camera -->
  <camera name="side_view" pos="0 -3 1" xyaxes="1 0 0 0 0.3 1"/>
  <camera name="tracking" pos="0 -2 1" mode="trackcom" target="ball"/>
</worldbody>
```

---

## 5. Question Checklist

When a user asks you to generate a MuJoCo scene, ask about any of these that are underspecified:

### Objects
- [ ] What shape? (sphere, box, capsule, mesh)
- [ ] What size? (radius, dimensions)
- [ ] What mass or density?
- [ ] What color/appearance?
- [ ] How many objects?

### Initial Conditions
- [ ] Starting position(s)?
- [ ] Starting velocity? (magnitude and direction)
- [ ] Starting orientation?

### Forces & Interactions
- [ ] What forces are applied? (gravity, thrust, springs)
- [ ] When are forces applied? (always, triggered, time-windowed)
- [ ] What direction and magnitude?
- [ ] Are there collisions? With what?

### Physical Properties
- [ ] Friction coefficients?
- [ ] Damping/drag?
- [ ] Restitution (bounciness)?
- [ ] Any constraints? (joints, limits)

### Simulation
- [ ] Duration? (seconds)
- [ ] Timestep? (default 0.002s)
- [ ] Gravity? (default Earth: 0 0 -9.81)

### Visualization
- [ ] Camera angle? (side, top, perspective)
- [ ] Render resolution?
- [ ] Background color?

---

## 6. Worked Example: DiSessa Ball Scenario

**Scenario**: A ball moves horizontally to the right. At a specific moment, a constant upward force is applied. What path does the ball follow?

- **Naive prediction**: Ball goes straight up (force "replaces" velocity)
- **Newtonian prediction**: Ball curves diagonally up-right (force "adds to" velocity)

### Complete MJCF

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

    <!-- Camera -->
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

### Simulation Logic (Python pseudocode)

```python
import mujoco

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Initial horizontal velocity (3 m/s rightward)
data.qvel[0] = 3.0

force_start_time = 0.5  # seconds
force_magnitude = 10.0  # Newtons upward
duration = 2.0

trajectory = []
while data.time < duration:
    # Apply upward force after trigger time
    if data.time >= force_start_time:
        data.ctrl[0] = force_magnitude
    else:
        data.ctrl[0] = 0.0

    mujoco.mj_step(model, data)

    trajectory.append({
        "t": data.time,
        "x": data.qpos[0],
        "y": data.qpos[1],
        "z": data.qpos[2],
        "vx": data.qvel[0],
        "vy": data.qvel[1],
        "vz": data.qvel[2],
    })
```

### Expected Result

The ball travels horizontally until t=0.5s, then follows a parabolic curve upward and to the right. The horizontal velocity is preserved (Newton's first law), while the vertical velocity increases linearly (F=ma).

---

## 7. Output Format

When generating MJCF XML:

1. **Always output complete, valid XML** — no partial snippets
2. **Include comments** explaining non-obvious choices
3. **Set reasonable defaults** for unspecified parameters
4. **Include cameras** for useful viewing angles
5. **Include sensors** for the quantities of interest
6. **Name everything** — bodies, joints, geoms, actuators, sensors
7. **Use SI units** — meters, kilograms, seconds, Newtons

```xml
<!-- Template -->
<mujoco model="DESCRIPTIVE_NAME">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <asset>
    <!-- textures and materials -->
  </asset>

  <worldbody>
    <light .../>
    <!-- ground, objects, cameras -->
  </worldbody>

  <actuator>
    <!-- forces and motors -->
  </actuator>

  <sensor>
    <!-- measurements -->
  </sensor>
</mujoco>
```
