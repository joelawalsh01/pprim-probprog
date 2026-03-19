# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**pprim-probprog** — a browser-based tool that uses Bayesian analysis-by-synthesis to find evidence for phenomenological primitive (p-prim) activation in students' physics predictions. Built on MuJoCo (physics) and Pyro (probabilistic programming).

The canonical scenario: a ball rolling horizontally receives an upward force. Students often predict "straight up" (force replaces velocity — the "force as mover" p-prim), while Newtonian mechanics predicts a diagonal curve (force adds to velocity). The tool infers what physical beliefs best explain a given prediction.

## Build & Run

### Docker (full stack)
```bash
docker compose up --build
# Frontend: http://localhost:3000
# Simulator API: http://localhost:8001
# Inference API: http://localhost:8002
```

### Local development (each service independently)
```bash
# Frontend (Vite dev server with HMR + proxy to local backends)
cd frontend && npm install && npm run dev

# Simulator
cd services/simulator && uv sync && uv run uvicorn src.main:app --host 0.0.0.0 --port 8001

# Inference
cd services/inference && uv sync && uv run uvicorn src.main:app --host 0.0.0.0 --port 8002
```

### Frontend build check
```bash
cd frontend && npm run build   # runs tsc + vite build
```

No test suite, linter, or CI exists yet.

## Architecture

Three Docker services, all behind an Nginx reverse proxy in the frontend container:

```
Browser (React/Vite/TypeScript :3000)
  ├─ /api/sim/*  → Simulator (FastAPI + MuJoCo :8001)  [nginx timeout: 300s]
  └─ /api/infer/* → Inference (FastAPI + Pyro :8002)    [nginx timeout: 600s]
```

- **Frontend** — 6-panel dashboard with project tabs. State lives in React + localStorage (no backend DB). Monaco editor (lazy-loaded, textarea fallback) for MJCF/Pyro code.
- **Simulator** — Loads MJCF XML, runs MuJoCo physics, returns trajectory points + base64 PNG frames. Uses OSMesa for offscreen rendering (`MUJOCO_GL=osmesa`).
- **Inference** — Runs Pyro SVI (default, fast) or MCMC/NUTS against an observed trajectory. Returns posterior distributions + plain-English p-prim interpretation.

### Dashboard panels (6-panel grid in `Dashboard.tsx`)
| Panel | Component | Role |
|-------|-----------|------|
| Top-left | `AnimationPanel` | Frame-by-frame playback of MuJoCo renders + canvas overlay for alternative trajectory |
| Top-center | `MujocoCodePanel` | MJCF XML editor → Simulate button → POST `/api/sim/simulate` |
| Top-right | `PyroCodePanel` | Dual tabs: Code (Pyro model editor) + P-Prims (structured config editor with Scan Code / Paste JSON) |
| Bottom-left | `PriorPosteriorPanel` | Mini histograms per inferred parameter with quantile stats |
| Bottom-center | `TrajectoryPanel` | Canvas 2D plot: Newtonian (blue) vs alternative (red dashed) trajectories |
| Bottom-right | `SummaryPanel` | Convergence plot, reasoning steps, findings, collapsible tabs |

### Data flow
1. User edits MJCF XML → clicks Simulate → frontend POSTs to `/api/sim/simulate` → gets trajectory + frames
2. User clicks Run Inference → frontend POSTs to `/api/infer/infer` with trajectory (or "naive" for alternative), model code, and p-prim config → gets posteriors + interpretation
3. Posterior-predictive sampling: `/api/infer/posterior-predictive` uses stored `_latest_posterior` global

### Key physics design choices
- Scene has **no gravity** (isolates force-vs-velocity question)
- Free joint with **damping=0** for clean physics
- `velocity_persistence` ∈ [0,1] is the key parameter: 0 = force replaces velocity (p-prim), 1 = Newtonian superposition

## Scenario System

Scenarios are registered in `App.tsx` (`SCENARIO_REGISTRY`) mapping scenario IDs to their MJCF XML, Pyro model code, P-Prim config, and visualization hints. Each scenario has:

1. **MJCF XML** — MuJoCo scene definition (in `services/simulator/src/examples/` and `frontend/src/defaults/mjcfExamples.ts`)
2. **Pyro model** — probabilistic model code (in `frontend/src/defaults/pyroModels.ts`)
3. **P-Prim config** — parameter→interpretation mappings (in `frontend/src/defaults/pprimConfigs.ts`)
4. **Visualization hints** — optional `VisualizationHints` (`showTube`, `showForceIndicator`) controlling AnimationPanel overlays. When omitted (e.g. user-created projects), AnimationPanel falls back to trajectory-shape heuristics.

Built-in scenarios: DiSessa Ball, Spiral Tube, Frictionless Push, Galileo's Drop.

New scenarios should be generated using `claude-mujoco-md/SCENARIO_GENERATION_GUIDE.md` which produces the first three artifacts. Set `visualHints` explicitly in the `SCENARIO_REGISTRY` entry to control which visual overlays appear.

## P-Prim Config System

The generalized interpretation system decouples parameter→p-prim mappings from code:

- **Frontend** uses camelCase (`pprimName`, `priorDist`); **backend** uses snake_case
- Conversion happens in `frontend/src/api.ts` via `pprimConfigToSnake()`
- Default config: `frontend/src/defaults/pprimConfigs.ts`
- Backend interpretation: `generate_interpretation_from_config()` in `services/inference/src/main.py`; falls back to hardcoded `generate_interpretation()` if no config provided
- `services/inference/src/inference.py` discovers parameter names dynamically from Pyro samples dict (no hardcoded parameter list)

## Conventions

- Python services use **`uv`** for dependency management (not pip/poetry)
- Frontend uses **npm** (not yarn/pnpm)
- No gravity in MuJoCo scenes by default
- localStorage key: `disessa-balls-projects` — frames are stripped before saving (too large)
- localStorage migration handles `pprimConfig: null` for old projects

## Important: Theoretical language

When writing user-facing text about student predictions, use "alternative" not "misconception." P-prims are not errors — they are context-sensitive intuitive knowledge elements that can be productive in many situations. The tool finds *evidence for* p-prim activation, not proof of cognitive states.
