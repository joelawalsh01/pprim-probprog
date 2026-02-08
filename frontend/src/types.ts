export interface TrajectoryPoint {
  t: number;
  x: number;
  y?: number;
  z: number;
  vx?: number;
  vy?: number;
  vz?: number;
}

export interface SimulationResult {
  trajectory: TrajectoryPoint[];
  naive_trajectory?: TrajectoryPoint[];
  frames?: string[];
  render_error?: string;
  metadata: {
    duration: number;
    num_points: number;
    initial_vx: number;
    force_start_time: number;
    force_magnitude: number;
    damping: number;
  };
}

export interface ParameterSummary {
  samples: number[];
  mean: number;
  std: number;
  median: number;
  q05: number;
  q95: number;
}

export interface InferenceResult {
  method: string;
  num_steps?: number;
  num_samples?: number;
  losses?: number[];
  posterior: Record<string, ParameterSummary>;
  observed_trajectory: { t: number; x: number; z: number }[];
  newtonian_trajectory: { t: number; x: number; z: number }[];
  model_code: string;
  interpretation: {
    findings: string[];
    summary: string;
    reasoning?: ReasoningStep[];
  };
}

export interface ReasoningStep {
  parameter: string;
  pprim_name?: string;
  prior: string;
  prior_mean: number;
  posterior_mean: number;
  posterior_std: number;
  shift: number;
  rule: string;
  thresholds: {
    condition: string;
    triggered: boolean;
    label: string;
  }[];
  conclusion: string;
}

export interface PPrimThreshold {
  min?: number | null;
  max?: number | null;
  label: string;
  conception: string; // "alternative" | "newtonian" | "mixed" | "neutral"
}

export interface PPrimMapping {
  parameter: string;
  pprimName: string;
  description: string;
  priorDist: string;
  priorMean: number;
  range: [number, number];
  color?: string;
  thresholds: PPrimThreshold[];
  lowConclusion: string;
  highConclusion: string;
}

export interface PPrimConfig {
  version: number;
  summaryTemplate?: string;
  mappings: PPrimMapping[];
}

export interface Project {
  id: string;
  name: string;
  mjcfXml: string;
  pyroCode: string;
  pprimConfig: PPrimConfig | null;
  simResult: SimulationResult | null;
  inferResult: InferenceResult | null;
}

export interface ExampleInfo {
  name: string;
  filename: string;
}

export interface ModelInfo {
  name: string;
  description: string;
  parameters: {
    name: string;
    prior: string;
    interpretation: string;
  }[];
  code: string;
}
