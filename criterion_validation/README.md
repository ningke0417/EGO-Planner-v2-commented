# Current-state split criterion validation

This directory isolates the geometric research question from EGO-Planner's
trajectory execution:

> Can current-state free-space overlap and pair visibility distinguish an
> intact formation that is locally constrained by a pillar from useful
> lower-cardinality subgroups?

The offline experiment does **not** modify or command the planner. The later
four-UAV criterion ROS node also remains shadow-only. Feedback is isolated in
an explicitly enabled one-way bridge so shadow-only and closed-loop runs cannot
be confused.

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

The four-UAV development/holdout result and the negative square-formation
result are recorded in `results/four_uav/REPORT.md`.

## Run the four-UAV ROS shadow experiment

From `swarm-playground/formation_ws`:

```bash
source devel/setup.zsh
roslaunch ego_planner formation_4uav_criterion_shadow.launch rviz:=true
```

RViz loads `/starconvex_shadow_4uav/markers` automatically. The other outputs
are `/starconvex_shadow_4uav/state`, `/starconvex_shadow_4uav/scores`, and
`/starconvex_shadow_4uav/summary`. For the square formation, `scores` is ordered
as perimeter edges `(0,1)`, `(1,2)`, `(2,3)`, `(3,0)`. The fixed 2+2 cut uses
`min(C12,C30)`; a displayed `SHADOW SPLIT G0={0,1} G1={2,3}` is a criterion
suggestion, not proof that EGO-Planner actually removed its formation coupling.

## Run the manual backend gate

Start directly in the real two-subgroup mode (still independent of the shadow
decision). In this mode the backend keeps only the relative pose inside
`{0,1}` and `{2,3}`; each subgroup centroid is free to translate laterally.
This is materially different from merely hiding the middle visualization
edge:

```bash
roslaunch ego_planner formation_4uav_criterion_shadow.launch \
  rviz:=true manual_split:=true
```

Or start intact and switch at runtime:

```bash
rostopic pub -1 /formation_backend/manual_split std_msgs/Bool "data: true"
rostopic pub -1 /formation_backend/manual_split std_msgs/Bool "data: false"
```

The RViz label has two separate lines: `SHADOW` is the criterion suggestion;
`BACKEND` is the state acknowledged by all four planners.  Thin cyan backend
edges show the active formation coupling; the cross-group `1-2` and `3-0`
cyan edges disappear
only when the actual backend is split.  Per-agent acknowledgments are published
on `/formation_backend/drone_0_group` through
`/formation_backend/drone_3_group`.

## Run the first automatic split gate

This launch closes only the `INTACT -> SPLIT` direction around the calibrated
single pillar:

```bash
roslaunch ego_planner formation_4uav_criterion_auto_split.launch rviz:=true
```

The transition is deliberately one-way for the duration of a run:

1. `AUTO_SPLIT ARMED`: the backend starts intact and no command has been sent.
2. `SHADOW WATCH`: the seam score is below `0.34`, but the formation center has
   not yet travelled the required `0.10 m` under that condition.
3. `AUTO_SPLIT COMMAND_SENT`: the persisted shadow state became `SPLIT` and a
   latched `true` command was published.
4. `AUTO_SPLIT CONFIRMED`: all planners acknowledged `G0,G0,G1,G1`; the cyan
   `1-2` and `3-0` backend edges are absent.

The controller never publishes `false`. Consequently, a later `SHADOW INTACT`
does not merge the backend during this experiment. Restarting the launch resets
it to intact. The controller status is also available on
`/formation_backend/auto_split_status`.

A headless integration run on 2026-08-02 observed `WATCH` at seam score `0.252`,
sent the split command on the next persisted update, received all four backend
acknowledgements in about `0.053 s`, and finished with all UAVs at their four
goal slots. The same run still contained four `swarm distance` warnings and one
`Failed to generate direction` warning, so reaching the goal is not yet a
safety proof. This establishes the software connection from the current-state
criterion to the real optimizer mode. It does not validate automatic merge,
prediction, wall geometry, or general obstacle layouts.

## Run the wall A/B comparison

The wall launch deliberately disables the pillar-specific StarConvex shadow
node. It tests one backend variable at a time: the original intact formation
cost versus the manual two-subgroup relative-pose cost.

Run the intact baseline and let it finish before starting the second command:

```bash
roslaunch ego_planner formation_4uav_backend_wall.launch \
  rviz:=true manual_split:=false
```

Then close it with `Ctrl-C` and run the split case:

```bash
roslaunch ego_planner formation_4uav_backend_wall.launch \
  rviz:=true manual_split:=true
```

Do not paste the two `roslaunch` commands onto one line. In RViz, use the
`BACKEND INTACT`/`BACKEND SPLIT` text and the thin cyan edges as the ground
truth for the optimizer state. In SPLIT, the cyan `1-2` and `3-0` edges must
be absent;
the colored trajectory curves themselves are history and do not prove which
couplings were active.

The default 5.0 m wall is a stress comparison, not a guaranteed pass/fail
separator. In local headless trials on 2026-08-01 both modes reached the goal,
while ROS warnings differed substantially: INTACT produced 65
`Failed to generate direction` messages and 24 `swarm distance` warnings;
SPLIT produced 1 and 3. A second 6.5 m trial also reached the goal in both
modes, with counts 10/11 for INTACT and 3/6 for SPLIT. These counts are
run-dependent and should be repeated, but they support a reduced-planning-
conflict claim, not an exclusive-success claim.

To inspect the longer wall without changing planner weights:

```bash
roslaunch ego_planner formation_4uav_backend_wall.launch \
  rviz:=true manual_split:=true wall_size_y:=6.5
```

## Run point-cloud criterion feedback on the wall

This launch no longer treats the wall as a cylinder. It voxelizes the latched
`/map_generator/global_cloud`, builds a safety-inflated obstacle field for the
StarConvex boundary rays, and builds a second, wider field for the `NARROW`
visibility class. The resulting boundary still goes through the original
StarConvex `Corridor` and `MeshTable` construction before overlap is sampled.

```bash
roslaunch ego_planner formation_4uav_criterion_auto_split_wall.launch \
  rviz:=true
```

The default wall is `0.5 x 5.0 x 3.1 m`; voxel resolution is `0.10 m`, planner
and criterion safety inflation are both `0.35 m`, and the StarConvex vertical
sampling band covers `0.5–2.5 m`. The controller remains one-way: it can split
but cannot merge. This wall-auto launch evaluates the criterion at `2 Hz`;
the shared shadow launch keeps a `1 Hz` default for the other experiments.

In a headless integration run on 2026-08-02, the frozen `0.34` seam threshold
first produced `WATCH` at score `0.243` and formation-center `x=-1.260 m`, then
confirmed `SPLIT` at score `0.246` and `x=-1.043 m`. The inflated wall front was
approximately `x=-0.60 m`, leaving about `0.44 m` before the safety boundary.
All four UAVs reached their goal slots, all four backend acknowledgements were
consistent, and no StarConvex computation, emergency-stop, heartbeat, or
certified-prefix errors occurred. The run still produced 45 direction-
generation failures and 15 inter-UAV-distance warnings. Therefore it validates
the point-cloud criterion-to-backend connection and a pre-boundary trigger in
one run; it does not yet establish safe or repeatable performance. The
threshold was frozen from the pillar study and has not been calibrated on wall
data.

A later single-variable trial raised only the criterion rate from `1 Hz` to
`2 Hz`. `WATCH` was logged at score `0.241`, center `x=-0.918 m`; the auto-split
bridge issued its command `0.492 s` later and all four planners acknowledged it
within `0.075 s`. Because the periodic state line is intentionally throttled to
`1 Hz`, the first printed `SPLIT` line followed at score `0.215`, center
`x=-0.670 m`; the command itself occurred between those two printed samples.
All four UAVs reached `x=6.0 m`. This run had zero emergency stops, zero
inter-UAV-distance warnings, zero direction-generation failures, and 33
successful collision-triggered replans. This is an improved run, but it does
not isolate the cause: earlier `1 Hz` trials triggered at substantially
different positions (`x=-1.043 m` and about `x=-0.58 m`). Repeated trials with
transition-time telemetry are still required before attributing the improvement
to the higher rate.

The backend visualizer also publishes subgroup diagnostics:

```bash
rostopic echo /formation_backend/subgroup_summary
```

`g0_distance` and `g1_distance` should stay near `1.0 m` after split if the two
fixed subgroups `{0,1}` and `{2,3}` are actually holding shape. `g0_same_side`
and `g1_same_side` show whether both members of a logical subgroup are on the
same side of the obstacle centerline. `g*_min_safety_clearance < 0` means at
least one member of that subgroup has entered the diagnostic safety envelope.
These diagnostics do not change planning; they separate “the criterion
triggered” from “the backend kept each subgroup coherent.”

The four-UAV launch also starts a shadow-only subgroup route-intent node.  It
uses only each UAV's current position, safety-inflated ray distances, the goal
direction, and exchanged subgroup candidate scores.  It does not use the wall
dimensions and does not command EGO-Planner.  After `BACKEND SPLIT`, RViz shows
a faint local-best arrow and a thick coordinated arrow for each subgroup.  Its
recordable outputs are:

```text
/subgroup_route_intent_4uav/intents
/subgroup_route_intent_4uav/summary
/subgroup_route_intent_4uav/markers
```

The single process is a centralized emulator for debugging, but the scoring is
structured so the same result can later be reconstructed from local candidate
vectors exchanged between distributed subgroup representatives.

The wall auto-split launch now enables a validation-only split-side guidance
term. It is inactive while the backend is `INTACT`. After the StarConvex
criterion triggers `SPLIT`, it softly penalizes `{0,1}` crossing to `y >= 0`
and `{2,3}` crossing to `y <= 0`, with a `0.35 m` margin around the wall
centerline. This is not automatic route assignment; it is a controlled
experiment to test whether criterion-triggered split becomes repeatable once
each fixed subgroup is kept on its own side.

The same wall launch also multiplies only the split-mode subgroup formation
cost by `4.0`. This strengthens the `{0,1}` and `{2,3}` relative-pose terms
without restoring the removed `1-2` coupling, so it should reduce temporary
`g0_distance`/`g1_distance` stretch while keeping the two-subgroup split.
