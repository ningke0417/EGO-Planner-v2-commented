# Current-state split trigger and nominal merge proxy

## Purpose

The previous experiment answered which connected bipartition is best after a
split is known to be necessary.  This experiment adds a state-machine question:

> Can current-state pair visibility and original-core StarConvex overlap
> distinguish a sustained obstruction from a harmless transient loss of
> relation quality?

There is still no predicted trajectory and no ROS/planner feedback.

## State machine

For every collision-free scan snapshot, all 20 connected bipartitions are
ranked.  For candidate partition `P`:

- `cut(P)` is the mean `overlap_min * visibility_weight` across edges cut by P;
- `internal(P)` is the mean of the same score on edges retained inside groups.

The best partition minimizes `cut(P)`.  Parameters were swept only on the
development cases.  The frozen rule is stored in
`config/current_state_trigger.yaml`:

```text
INTACT -> SPLIT
  best cut <= 0.46
  and best internal mean >= 0.50
  continuously over 0.30 m of formation-centre travel

SPLIT -> MERGE (proxy only)
  cut score of the frozen active partition >= 0.75
  continuously over 0.30 m of travel
```

The two thresholds provide hysteresis.  Persistence is expressed in travelled
distance rather than frame count so results are comparable between the 0.2 m
development scan and 0.1 m holdout scan.

## Development and frozen holdout result

The development sweep evaluated 3,780 combinations.  A non-isolated plateau
achieved zero case-level errors: split threshold 0.43--0.50 and persistence
0.3--0.4 m.  The middle values were frozen before inspecting holdout behavior.

| dataset | obstructed cases detected | passable cases falsely split | trigger lead |
|---|---:|---:|---:|
| development | 9/9 | 0/6 | 0.0 m |
| held-out radii/offsets | 8/8 | not available | 0.0--0.1 m |

The holdout set contains eight obstructed cases but no passable lateral-offset
case, so it validates missed-trigger behavior but cannot independently validate
the false-trigger rate.

The nominal rigid scan returns above the merge threshold in all 17 obstructed
cases, between centre x = 2.2 and 3.0 m.  This is only a recovery proxy: after a
real split the subgroup positions differ from the rigid scan, so these numbers
do not prove safe online merging.

## Why visibility alone is insufficient

A visibility-only state machine can achieve the same case-level split timing
on this simple vertical-cylinder dataset.  It does not reliably choose the
right partition because many candidate cuts contain the same binary
VISIBLE/NARROW/OCCLUDED values:

| partition metric | development top-1 valid | holdout top-1 valid | combined |
|---|---:|---:|---:|
| visibility weight only | 3/9 | 2/8 | 5/17 |
| original StarConvex overlap × visibility | 9/9 | 8/8 | 17/17 |

Thus, in this experiment visibility largely supplies the timing signal, while
continuous StarConvex overlap breaks visibility ties and supplies the useful
subgroup choice.  This is a concrete reason to combine them.

## Limitation that matters most

The frozen no-preview trigger has almost no safety lead: it fires at, or at
most 0.1 m before, the offline oracle first says the rigid formation cannot
make the next 0.5 m translation.  This is enough for the teacher's initial
feasibility question, but not enough evidence for smooth closed-loop flight.
At nontrivial speed, the planner may need earlier information, a reduced speed
near WATCH state, or eventually a short preview window.

## Decision

The next justified integration is ROS **shadow mode only**:

- publish INTACT/WATCH/SPLIT/MERGE_CANDIDATE state;
- publish the frozen candidate partition and persistence accumulator;
- record bags and compare decisions against actual planner progress;
- do not yet remove formation constraints or assign subgroup routes.

Planner feedback should remain disabled until shadow runs show stable timing
under real map noise and asynchronous UAV updates.

