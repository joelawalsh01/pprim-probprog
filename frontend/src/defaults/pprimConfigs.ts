import type { PPrimConfig } from '../types';

export const DEFAULT_SPIRAL_TUBE_PPRIM_CONFIG: PPrimConfig = {
  version: 1,
  summaryTemplate: undefined,
  mappings: [
    {
      parameter: 'path_curvature_persistence',
      pprimName: 'Curvilinear impetus',
      description:
        'Controls whether the ball continues curving after leaving the tube. ' +
        'At 1, the ball "remembers" the curved path and continues along an arc — as if the tube imparted a curving tendency. ' +
        'At 0, the ball exits along a straight tangent line (Newtonian: no force, no curve).',
      priorDist: 'Beta(2,2)',
      priorMean: 0.5,
      range: [0, 1],
      color: '#7aa2f7',
      thresholds: [
        { max: 0.3, label: 'Straight tangent exit (Newtonian)', conception: 'newtonian' },
        { min: 0.3, max: 0.7, label: 'Partial curving — mixed beliefs', conception: 'mixed' },
        { min: 0.7, label: "Continues curving ('curvilinear impetus' p-prim active)", conception: 'alternative' },
      ],
      lowConclusion:
        'Consistent with Newtonian mechanics — the ball exits along a straight tangent once the constraining channel ends.',
      highConclusion:
        "Strong evidence for the 'curvilinear impetus' p-prim — the person expects the ball to continue curving as if the tube imparted a lasting curved motion.",
    },
    {
      parameter: 'exit_speed',
      pprimName: 'Exit speed',
      description:
        'The speed of the ball as it exits the curved channel. ' +
        'This is a nuisance parameter that the model adjusts to match the observed trajectory scale.',
      priorDist: 'LogNormal(0.5,0.5)',
      priorMean: 1.87,
      range: [0, 5],
      color: '#9ece6a',
      thresholds: [],
      lowConclusion: 'The model found a low exit speed best reproduces the observed trajectory.',
      highConclusion: 'The model found a high exit speed best reproduces the observed trajectory.',
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

export const DEFAULT_FRICTIONLESS_PUSH_PPRIM_CONFIG: PPrimConfig = {
  version: 1,
  summaryTemplate: undefined,
  mappings: [
    {
      parameter: 'intrinsic_damping',
      pprimName: 'Continuous force / Dying away',
      description:
        'Controls whether the object slows down after being pushed on a frictionless surface. ' +
        'At 0, the object maintains constant velocity forever (Newtonian: no friction means no deceleration). ' +
        'High values mean the object slows and stops on its own — as if motion naturally "dies away" without sustained force.',
      priorDist: 'LogNormal(0,1)',
      priorMean: 1.65,
      range: [0, 10],
      color: '#f7768e',
      thresholds: [
        { max: 0.3, label: 'No damping — constant velocity (Newtonian)', conception: 'newtonian' },
        { min: 0.3, max: 2.0, label: 'Moderate damping — partial slowing', conception: 'mixed' },
        { min: 2.0, label: "Motion dies away ('continuous force' / 'dying away' p-prim active)", conception: 'alternative' },
      ],
      lowConclusion:
        "Consistent with Newton's first law — on a frictionless surface, a pushed object maintains constant velocity indefinitely.",
      highConclusion:
        "Strong evidence for the 'dying away' p-prim — the person expects the object to slow and stop even without friction, as if motion requires sustained force.",
    },
    {
      parameter: 'push_force',
      pprimName: 'Push strength',
      description:
        'The magnitude of the initial push force. This determines the initial velocity ' +
        'and is adjusted by the model to match the scale of the observed trajectory.',
      priorDist: 'LogNormal(1,0.5)',
      priorMean: 3.08,
      range: [0, 20],
      color: '#9ece6a',
      thresholds: [],
      lowConclusion: 'The model found a low push force best reproduces the observed motion.',
      highConclusion: 'The model found a high push force best reproduces the observed motion.',
    },
    {
      parameter: 'mass',
      pprimName: 'Perceived mass',
      description:
        'The perceived mass of the object. Together with push_force, determines the initial velocity (v = F*t/m).',
      priorDist: 'LogNormal(0,0.5)',
      priorMean: 1.13,
      range: [0, 5],
      color: '#e0af68',
      thresholds: [],
      lowConclusion: 'The inferred mass is low, implying faster initial motion.',
      highConclusion: 'The inferred mass is high, implying slower initial motion.',
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

export const DEFAULT_GALILEO_DROP_PPRIM_CONFIG: PPrimConfig = {
  version: 1,
  summaryTemplate: undefined,
  mappings: [
    {
      parameter: 'mass_gravity_coupling',
      pprimName: "Ohm's p-prim (heavier falls faster)",
      description:
        'Controls whether mass affects the rate of falling. ' +
        'At 0, both objects fall at the same rate regardless of mass (Newtonian: a = g, independent of mass). ' +
        'At 1, heavier objects fall faster — as if more mass means more "oomph" pulling it down.',
      priorDist: 'Beta(2,2)',
      priorMean: 0.5,
      range: [0, 1],
      color: '#7aa2f7',
      thresholds: [
        { max: 0.3, label: 'Mass-independent fall rate (Newtonian)', conception: 'newtonian' },
        { min: 0.3, max: 0.7, label: 'Partial mass effect — mixed beliefs', conception: 'mixed' },
        { min: 0.7, label: "Heavier falls faster (Ohm's p-prim active)", conception: 'alternative' },
      ],
      lowConclusion:
        'Consistent with Galileo and Newton — all objects fall at the same rate in a vacuum, regardless of mass.',
      highConclusion:
        "Strong evidence for Ohm's p-prim — the person expects heavier objects to fall faster, as if mass directly drives falling speed.",
    },
    {
      parameter: 'g_effective',
      pprimName: 'Effective gravity',
      description:
        'The effective gravitational acceleration. Adjusted by the model to match the overall timescale of the observed drop.',
      priorDist: 'LogNormal(2.3,0.3)',
      priorMean: 10.49,
      range: [0, 20],
      color: '#9ece6a',
      thresholds: [],
      lowConclusion: 'The inferred gravity is low — the observed drop is slower than expected.',
      highConclusion: 'The inferred gravity is high — the observed drop is faster than expected.',
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

export const DEFAULT_DISESSA_PPRIM_CONFIG: PPrimConfig = {
  version: 1,
  summaryTemplate: undefined,
  mappings: [
    {
      parameter: 'velocity_persistence',
      pprimName: 'Force as mover',
      description:
        'Controls what happens to existing velocity when a new force is applied. ' +
        'At 0, the force completely replaces the current velocity — like kicking a stationary ball, where it goes where you kick it. ' +
        'At 1, the force adds to the current velocity (Newtonian superposition).',
      priorDist: 'Beta(2,2)',
      priorMean: 0.5,
      range: [0, 1],
      color: '#7aa2f7',
      thresholds: [
        { max: 0.3, label: "Force replaces velocity ('force as mover' p-prim active)", conception: 'alternative' },
        { min: 0.3, max: 0.7, label: 'Mixed beliefs — partial persistence', conception: 'mixed' },
        { min: 0.7, label: 'Force adds to velocity (Newtonian superposition)', conception: 'newtonian' },
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
        'High values mean horizontal motion dies quickly even without a new force — like the everyday experience ' +
        'that a ball kicked across grass slows down on its own. Low values mean motion persists (Newton\'s first law).',
      priorDist: 'LogNormal(0,1)',
      priorMean: 1.65,
      range: [0, 10],
      color: '#f7768e',
      thresholds: [
        { max: 0.5, label: 'Low damping — motion persists (Newtonian)', conception: 'newtonian' },
        { min: 0.5, max: 2.0, label: 'Moderate damping', conception: 'mixed' },
        { min: 2.0, label: "High damping — motion dies quickly ('dying away' p-prim active)", conception: 'alternative' },
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
