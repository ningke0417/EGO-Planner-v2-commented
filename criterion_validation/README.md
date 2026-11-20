# Current-state split criterion validation

This directory isolates the geometric research question from EGO-Planner's
trajectory execution:

> Can current-state free-space overlap and pair visibility distinguish an
> intact formation that is locally constrained by a pillar from useful
> lower-cardinality subgroups?

The offline experiment does **not** modify or command the planner. The former
four-UAV ROS shadow and feedback entry points have been retired; this directory
retains the offline experiment and the shared StarConvex geometry adapter used
by the current eight-UAV scenarios.

## Scope of the first experiment

- eight-UAV ranking experiments plus a controlled four-UAV lateral 2+2 test;
- one deterministic vertical cylinder;
- current positions only (`tau = 0`), with no predicted trajectory;
- explicit, separately configurable candidate relation edges;
- machine-readable output suitable for later plots and repeatability checks.

The candidate relation graph is not part of the original EGO-Planner
formation cost.  The original optimizer penalizes displacement from desired
formation positions relative to a virtual formation center.  Edges in this
experiment are therefore a research hypothesis and must be evaluated rather
than treated as ground truth.

## Files

- `config/formation_8_rectangle.yaml`: original eight-UAV offsets plus a
  separately labelled candidate grid graph.
- `config/formation_4_lateral_line.yaml`: four-UAV chain with one admissible
  connected 2+2 split.
- `config/current_state_trigger_4uav.yaml`: frozen four-UAV shadow parameters.
- `config/single_pillar.yaml`: deterministic geometry and scan domain.
- `scripts/check_inputs.py`: input validation only.
- `scripts/run_static_scan.py`: raw current-state overlap and visibility scan.
- `scripts/evaluate_partitions.py`: threshold sweep against an independent
  local-motion feasibility label.
- `criterion/geometry.py`: analytic 2-D radial geometry implementation.

## Run the input check

From this repository root:

```bash
python3 criterion_validation/scripts/check_inputs.py \
  --formation criterion_validation/config/formation_8_rectangle.yaml \
  --scenario criterion_validation/config/single_pillar.yaml
```

The current implementation computes raw current-state star-space, overlap and
visibility values behind these stable input files.  It does not turn those
values into split or merge decisions.

## Run the raw current-state scan

```bash
python3 criterion_validation/scripts/run_static_scan.py \
  --formation criterion_validation/config/formation_8_rectangle.yaml \
  --scenario criterion_validation/config/single_pillar.yaml \
  --csv-out /tmp/single_pillar_scores.csv \
  --summary-out /tmp/single_pillar_scores.json
```

This output contains both intersection-over-smaller-space and IoU overlap,
inflated-pillar segment clearance, visibility class, and the corresponding
weighted composite values.  It contains no split threshold or topology
decision.

## Evaluate candidate graph partitions

```bash
python3 criterion_validation/scripts/evaluate_partitions.py \
  --formation criterion_validation/config/formation_8_rectangle.yaml \
  --scenario criterion_validation/config/single_pillar.yaml \
  --scores /tmp/single_pillar_scores.csv \
  --csv-out /tmp/single_pillar_partitions.csv \
  --summary-out /tmp/single_pillar_partitions.json
```

The local-motion oracle used here is an offline label only.  It checks a set
of short rigid translations against inflated pillar geometry and is never fed
back into the overlap/visibility score.  Thresholds are swept rather than
selected, so this stage cannot silently tune one threshold to one picture.
Each component is evaluated independently; this label does not yet solve
joint route assignment or inter-component collision avoidance.

## Recorded single-pillar result

The first complete current-state experiment is stored under
`results/single_pillar/`.  Read `results/single_pillar/REPORT.md` before using
its scores: the result exposes important limitations and does not authorize
closed-loop topology control.

The second-stage screening configuration is
`config/multi_configuration_sweep.yaml`.  It varies pillar radius and lateral
alignment before any candidate bipartition ranking is attempted.

Development/holdout connected-bipartition results are recorded under
`results/bipartition_analysis/`.  The positive ranking result is conditional;
the report requires a comparison with the real 3-D StarConvex construction
before any ROS shadow integration.

That original-core comparison is now recorded in
`results/starconvex_comparison/REPORT.md`.  It reuses the unmodified StarConvex
`Corridor` and `MeshTable` code through the standalone adapter under
`starconvex_adapter/`.  The result validates conditional subgroup ranking, not
the split trigger or planner feedback; those remain separate gates.

The next offline gate is recorded in `results/trigger_analysis/REPORT.md`.
It adds a distance-persistent, hysteretic current-state trigger, freezes its
parameters on development cases, evaluates them on held-out radii/offsets, and
compares the combined criterion against visibility alone.  Its merge output is
explicitly a nominal rigid-scan proxy, not a closed-loop merge validation.

## Current ROS scenarios

The former four-UAV and six-UAV ROS experiment launch files and standalone
four-/six-UAV build targets have been retired from the current tree. The
single-pillar offline configurations, scripts and recorded results above remain
available. Historical ROS observations refer to the versions used for those
experiments; they are not current launch instructions.

The maintained ROS scenarios are the eight-UAV fixed-wall and sparse-random
launches documented in [the repository README](../README.md). Both default to
distributed mode; eight-UAV centralized mode remains available explicitly.
Build instructions, protocol details and regression checks are in
[the distributed formation guide](../docs/distributed-formation.md).

The shared geometry headers under `starconvex_adapter/include/` are still used
by the current distributed agent and eight-UAV criterion nodes. Keep this
adapter when removing old experiment entry points.
