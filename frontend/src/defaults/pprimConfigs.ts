import type { PPrimConfig } from '../types';

export const DEFAULT_DISESSA_PPRIM_CONFIG: PPrimConfig = {
  version: 1,
  summaryTemplate: undefined,
  mappings: [
    {
      parameter: 'velocity_persistence',
      pprimName: 'Force as mover',
      description:
        'Controls what happens to existing velocity when a new force is applied. ' +
        'At 0, the force completely replaces the current velocity (the ball forgets it was moving horizontally). ' +
        'At 1, the force adds to the current velocity (Newtonian superposition).',
      priorDist: 'Beta(2,2)',
      priorMean: 0.5,
      range: [0, 1],
      color: '#7aa2f7',
      thresholds: [
        { max: 0.3, label: "Force replaces velocity (alternative conception / 'force as mover' p-prim)", conception: 'alternative' },
        { min: 0.3, max: 0.7, label: 'Mixed beliefs — partial persistence', conception: 'mixed' },
        { min: 0.7, label: "Force adds to velocity (Newtonian / 'force as deflector')", conception: 'newtonian' },
      ],
      lowConclusion:
        "Strong evidence for the 'force as mover' p-prim — the person's predicted trajectory is best explained by a model where force replaces existing velocity.",
      highConclusion:
        'Beliefs broadly consistent with Newtonian mechanics — force adds to existing motion.',
    },
    {
      parameter: 'lateral_damping',
      pprimName: 'Dying away',
      description:
        'Controls how quickly horizontal velocity decays over time, independent of velocity_persistence. ' +
        'High values mean horizontal motion dies quickly even without a new force — like an implicit belief ' +
        'that objects naturally stop moving (an Aristotelian intuition). Low values mean motion persists (Newton\'s first law).',
      priorDist: 'LogNormal(0,1)',
      priorMean: 1.65,
      range: [0, 10],
      color: '#f7768e',
      thresholds: [
        { max: 0.5, label: 'Low damping — motion persists (Newtonian)', conception: 'newtonian' },
        { min: 0.5, max: 2.0, label: 'Moderate damping', conception: 'mixed' },
        { min: 2.0, label: 'High damping — motion dies quickly (Aristotelian)', conception: 'alternative' },
      ],
      lowConclusion:
        "Consistent with Newton's first law — objects in motion stay in motion.",
      highConclusion:
        'The person expects horizontal motion to dissipate, as if objects naturally come to rest unless actively pushed.',
    },
    {
      parameter: 'force_magnitude',
      pprimName: 'Force strength',
      description:
        'The perceived strength of the applied force. Interacts with mass to determine vertical acceleration (a = F/m). ' +
        'The inference adjusts this to match the vertical component of the observed trajectory.',
      priorDist: 'LogNormal(1,0.5)',
      priorMean: 3.08,
      range: [0, 20],
      color: '#9ece6a',
      thresholds: [],
      lowConclusion: 'The model found a low force magnitude best reproduces the observed vertical motion.',
      highConclusion: 'The model found a high force magnitude best reproduces the observed vertical motion.',
    },
    {
      parameter: 'mass',
      pprimName: 'Perceived mass',
      description:
        'The perceived mass of the object. Together with force_magnitude, this determines vertical acceleration. ' +
        'Mass and force are partially degenerate (only their ratio matters for acceleration), so the posterior may be broad.',
      priorDist: 'LogNormal(0,0.5)',
      priorMean: 1.13,
      range: [0, 5],
      color: '#e0af68',
      thresholds: [],
      lowConclusion: 'The inferred mass is low, implying higher perceived acceleration.',
      highConclusion: 'The inferred mass is high, implying lower perceived acceleration.',
    },
    {
      parameter: 'sigma',
      pprimName: 'Observation noise',
      description: 'The observation noise — how tightly the model must match the observed trajectory.',
      priorDist: 'LogNormal(-2,1)',
      priorMean: 0.2,
      range: [0, 1],
      color: '#bb9af7',
      thresholds: [],
      lowConclusion: 'Low noise — the model fits the trajectory tightly.',
      highConclusion: 'High noise — the model has a loose fit to the trajectory.',
    },
  ],
};
