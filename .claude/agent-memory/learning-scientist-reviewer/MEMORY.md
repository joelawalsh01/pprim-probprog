# Learning Scientist Reviewer Memory

## Key Theoretical Issues Found
- Project conflates p-prims with alternative conceptions throughout; see `pprim-terminology.md`
- "Force as deflector" is used as a p-prim label but is not a standard diSessa p-prim
- "Naive" and "Aristotelian" labels are deficit-model language; should be "alternative" and "everyday-experience-based"
- Tool infers model parameters, not p-prims directly; docs should say "p-prim signatures" not "extracted p-prims"

## Terminology Decisions
- "Cognitive physics parameters" is the best term in the docs; use consistently
- "Alternative prediction" preferred over "alternative conception" when describing trajectories
- "(correct)" after "Newtonian" should be dropped everywhere

## Files Reviewed (2026-02-08)
- `claude-mujoco-md/SCENARIO_GENERATION_GUIDE.md` -- generation guide for LLM scenario creation
- `README.md` -- project overview and theoretical background
- `frontend/src/defaults/pprimConfigs.ts` -- default p-prim config for DiSessa ball
- `services/inference/src/models.py` -- actual Pyro model code (has DEFAULT_MODEL_CODE string)
- `frontend/src/components/SummaryPanel.tsx` -- how threshold/conclusion text surfaces in UI

## What Works Well
- Analysis-by-synthesis framing is clear and compelling
- "A student can be 30% Newtonian" is excellent for communicating continuous measurement
- Threshold/conception three-band system (alternative/mixed/newtonian) is principled
- Worked example (diSessa ball) is well-chosen and from the literature

## Detailed Notes
- See `pprim-terminology.md` for full terminology analysis
- See `deficit-language-locations.md` for all locations needing language changes
