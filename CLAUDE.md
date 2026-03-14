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
  ├─ /api/sim/*  → Simulator (FastAPI + MuJoCo :8001)
  └─ /api/infer/* → Inference (FastAPI + Pyro :8002)
```

- **Frontend** — 6-panel dashboard with project tabs. State lives in React + localStorage (no backend DB). Monaco editor for MJCF/Pyro code, Plotly for visualizations.
- **Simulator** — Loads MJCF XML, runs MuJoCo physics, returns trajectory points + base64 PNG frames. Uses OSMesa for offscreen rendering (`MUJOCO_GL=osmesa`).
- **Inference** — Runs Pyro SVI (default, fast) or MCMC/NUTS against an observed trajectory. Returns posterior distributions + plain-English p-prim interpretation.

### Data flow
1. User edits MJCF XML → clicks Simulate → frontend POSTs to `/api/sim/simulate` → gets trajectory + frames
2. User clicks Run Inference → frontend POSTs to `/api/infer/infer` with trajectory (or "naive" for alternative), model code, and p-prim config → gets posteriors + interpretation

### Key physics design choices
- Scene has **no gravity** (isolates force-vs-velocity question)
- Free joint with **damping=0** for clean physics
- `velocity_persistence` ∈ [0,1] is the key parameter: 0 = force replaces velocity (p-prim), 1 = Newtonian superposition

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
- localStorage migration handles `pprimConfig: null` for old projects
- Scenario generation guide at `claude-mujoco-md/SCENARIO_GENERATION_GUIDE.md` generates all three outputs (MJCF + Pyro model + P-Prim config JSON)

## Important: Theoretical language

When writing user-facing text about student predictions, use "alternative" not "misconception." P-prims are not errors — they are context-sensitive intuitive knowledge elements that can be productive in many situations. The tool finds *evidence for* p-prim activation, not proof of cognitive states.
