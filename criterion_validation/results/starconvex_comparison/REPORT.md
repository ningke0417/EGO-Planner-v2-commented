# Original StarConvex comparison

## Question

Does the positive 2-D prototype result survive when pair overlap is recomputed
with the original StarConvex sphere-flipping/QuickHull construction and its
original mesh-membership query?

This remains a current-state (`tau = 0`) offline experiment.  It neither sends
topology decisions to ROS nor changes EGO-Planner trajectories.

## What is original and what is adapted

Used unchanged from `/home/robot/Codes/StarConvex`:

- `FastLab::Corridor::build` from the compiled `libstar_cvx.a`;
- its 500 m sphere-flipping radius and 0.02 QuickHull precision;
- the two-pass construction sequence from `FreeSpace::create_convexhull`;
- `FastLab::MeshTable::createTable` and `isInStarHull`.

Supplied by the offline adapter:

- reconstruction of one inflated vertical cylinder and eight UAV poses;
- the ray batches and vertical layers used by `FreeSpaceFactory`;
- analytic cylinder ray casting, quantized to the original 0.1 m map-ray step;
- common-grid volume sampling for pair overlap.

This is therefore a test of the original StarConvex **geometric core**, not a
claim that the complete StarConvex ROS mapping pipeline has been transplanted.

## Dataset

Only decision-relevant snapshots and nearby controls are evaluated:

| set | blocked snapshots with a valid split | nearby intact controls | total |
|---|---:|---:|---:|
| development | 9 | 18 | 27 |
| held out radii/offsets | 8 | 16 | 24 |

The held-out radii (0.9, 1.1 m) and lateral offsets (-0.75, -0.25, 0.25,
0.75 m) were not used to choose the ranking rule.

## Result

The previously selected rule remains:

1. enumerate 20 connected bipartitions, with at least two UAVs per group;
2. score a partition by the mean `overlap_min * visibility_weight` of the
   relation edges it cuts;
3. prefer the smallest mean cut score.

At the offline oracle distance of 0.5 m:

| overlap implementation | development top-1 | holdout top-1 | combined |
|---|---:|---:|---:|
| analytic 2-D prototype | 9/9 | 8/8 | 17/17 |
| original 3-D StarConvex core | 9/9 | 8/8 | 17/17 |

For all 17 blocked snapshots, the original 3-D core selected the same top
partition as the 2-D prototype.  Across the 510 evaluated relation rows, the
Pearson correlation between prototype and original-core composite scores was
0.9991 on development and 0.9988 on holdout; mean absolute differences were
0.0118 and 0.0108 respectively.

The result is not a degenerate empty-mesh artifact: generated meshes contained
50--108 triangles on development and 52--106 on holdout.  Input boundary
clouds contained 116--313 and 133--312 points respectively.

Changing the development-set horizontal volume-sampling resolution from 0.25
m to 0.20 m and 0.15 m preserved the 9/9 top-1 result.  The main conclusion is
therefore not caused by the chosen 0.25 m integration grid.

## Interpretation

This closes one uncertainty: the earlier positive grouping result was not an
artifact of having replaced StarConvex with a tiny 2-D radial class.  In this
vertical-pillar, fixed-altitude setting, the simple model and the original 3-D
core encode almost the same relation ordering.

It does **not** yet establish when to trigger a split.  Ranking a partition
conditional on "a split is needed" is different from distinguishing normal
flight, impending blockage, and safe merge.  The nearby controls are retained
for that next trigger/hysteresis experiment; no partition should be applied to
them merely because a ranking always has a first place.

It also does not prove online timing, subgroup trajectory feasibility,
inter-subgroup collision avoidance, merge behavior, or fully distributed
operation.

## Decision

The original-core comparison passes its gate.  The next safe step is an
offline current-state split-trigger study using the blocked snapshots and
controls, with separate split/merge thresholds and persistence.  Only after
that rule shows low false-trigger rates should the exact same calculation be
published in ROS shadow mode.  Planner feedback remains out of scope until the
shadow output is stable.

