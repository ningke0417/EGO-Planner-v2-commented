# Four-UAV current-state criterion

## Controlled formation

Four UAVs form one lateral chain at `y = [-1.5, -0.5, 0.5, 1.5]` while
travelling in `+x`. Requiring two connected members per subgroup leaves one
admissible split: `{0,1}|{2,3}`. This is a controlled 2+2 feasibility test,
not evidence that the current-state criterion can choose arbitrary route-side
assignments.

The original StarConvex `Corridor::build` two-pass construction and
`MeshTable::isInStarHull` test are linked from the unchanged checkout at
`/home/robot/Codes/StarConvex`. Visibility uses the inflated single-pillar
geometry. The composite edge score is
`overlap_min * visibility_weight`.

## Frozen trigger result

Parameters were selected on the development sweep and then used unchanged on
the holdout sweep:

- split threshold: `0.34`
- merge threshold: `0.75`
- spatial persistence: `0.10 m`

At the 0.5 m local-motion oracle distance, the frozen trigger detected all
3/3 development cases and all 4/4 holdout cases having a useful 2+2 split. It
did not split in the remaining 12 development and 4 holdout comparison cases.
The minimum lead distance is still zero in some cases, which is an expected
limitation of the teacher-requested no-preview experiment.

The square 2x2 formation was also tested and failed the holdout route grouping:
the smallest current-state graph cut can be the front/back partition instead
of the lower/upper route partition. Threshold tuning cannot resolve that
direction ambiguity without route information or preview.

## ROS shadow integration

`formation_4uav_criterion_shadow.launch` starts four EGO-Planner agents, one
fixed pillar, and `starconvex_shadow_4uav_node`. The node publishes only state,
scores, summary, and RViz markers. It has no publisher or subscriber connected
to planner topology, formation weights, targets, or trajectories. The launch
defaults to `max_vel=0.3 m/s` so the state transitions remain observable.

A headless smoke run produced the sequence
`INTACT -> WATCH -> SPLIT -> MERGE_CANDIDATE -> INTACT`. Original StarConvex
calculation took about 0.02--0.07 seconds per update on this machine.

## Manual two-subgroup backend gate

The planner now has a deliberately manual backend switch for isolating the
next question from the StarConvex trigger.  In `INTACT`, the formation cost
uses all four agents.  In `SPLIT`, it uses `{0,1}` for drones 0 and 1 and
`{2,3}` for drones 2 and 3.  Swarm collision checking still considers all four
agents, so this switch does not disable inter-group collision avoidance and it
does not prescribe a left/right path.

The fixed-pillar launch uses `0.35 m` for both the optimizer obstacle clearance
and the requested occupancy-map inflation.  At `0.1 m` map resolution the
occupancy boundary is conservatively rounded upward to four voxels (`0.4 m`).
The former inflation expression mixed metres and voxel counts and was fixed in
both map update paths.

A headless smoke test with `manual_split:=true` reached the four final states
at `x = 6.0`, with final lateral positions approximately
`[-1.5, -0.5, 0.5, 1.5]`.  The four acknowledged backend states were
`G0,G0,G1,G1`; a subsequent manual merge produced four `INTACT` acknowledgments.
This proves that the two-group formation-cost mode is executed and can finish
this single-pillar scenario.  It does **not** yet prove rigid intra-group shape:
the run contained recoverable predicted pair-distance warnings around
`0.35--0.40 m`.  Quantifying and correcting that deformation is the next gate.
