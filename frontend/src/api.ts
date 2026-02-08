import type { SimulationResult, InferenceResult, ExampleInfo, ModelInfo, PPrimConfig } from './types';

const SIM_BASE = '/api/sim';
const INFER_BASE = '/api/infer';

async function fetchJSON<T>(url: string, options?: RequestInit): Promise<T> {
  const res = await fetch(url, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || `HTTP ${res.status}`);
  }
  return res.json();
}

// Simulator API

export async function getExamples(): Promise<{ examples: ExampleInfo[] }> {
  return fetchJSON(`${SIM_BASE}/examples`);
}

export async function getExample(name: string): Promise<{ name: string; mjcf_xml: string }> {
  return fetchJSON(`${SIM_BASE}/examples/${name}`);
}

export async function simulate(params: {
  mjcf_xml: string;
  duration?: number;
  initial_vx?: number;
  force_start_time?: number;
  force_magnitude?: number;
  damping?: number;
  render_frames?: boolean;
  max_frames?: number;
  include_naive?: boolean;
}): Promise<SimulationResult> {
  return fetchJSON(`${SIM_BASE}/simulate`, {
    method: 'POST',
    body: JSON.stringify(params),
  });
}

// Inference API

export async function getModels(): Promise<{ models: ModelInfo[] }> {
  return fetchJSON(`${INFER_BASE}/models`);
}

/** Convert a PPrimConfig (camelCase frontend) to snake_case for the backend. */
function pprimConfigToSnake(config: PPrimConfig) {
  return {
    version: config.version,
    summary_template: config.summaryTemplate ?? null,
    mappings: config.mappings.map(m => ({
      parameter: m.parameter,
      pprim_name: m.pprimName,
      description: m.description,
      prior_dist: m.priorDist,
      prior_mean: m.priorMean,
      range: m.range,
      color: m.color ?? null,
      thresholds: m.thresholds.map(t => ({
        min: t.min ?? null,
        max: t.max ?? null,
        label: t.label,
        conception: t.conception,
      })),
      low_conclusion: m.lowConclusion,
      high_conclusion: m.highConclusion,
    })),
  };
}

export async function runInference(params: {
  trajectory_type?: string;
  observed_trajectory?: { x: number; z: number }[];
  model_code?: string;
  method?: string;
  num_steps?: number;
  num_samples?: number;
  learning_rate?: number;
  pprim_config?: PPrimConfig | null;
}): Promise<InferenceResult> {
  const { pprim_config, ...rest } = params;
  const body: Record<string, unknown> = { ...rest };
  if (pprim_config) {
    body.pprim_config = pprimConfigToSnake(pprim_config);
  }
  return fetchJSON(`${INFER_BASE}/infer`, {
    method: 'POST',
    body: JSON.stringify(body),
  });
}

export async function priorPredictive(params?: {
  num_samples?: number;
}): Promise<{ trajectories: { t: number; x: number; z: number }[][]; num_samples: number }> {
  return fetchJSON(`${INFER_BASE}/prior-predictive`, {
    method: 'POST',
    body: JSON.stringify(params || {}),
  });
}

export async function posteriorPredictive(params?: {
  num_samples?: number;
}): Promise<{ trajectories: { t: number; x: number; z: number }[][]; num_samples: number }> {
  return fetchJSON(`${INFER_BASE}/posterior-predictive`, {
    method: 'POST',
    body: JSON.stringify(params || {}),
  });
}
