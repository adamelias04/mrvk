# Predictor evaluation harness

Tooling for measuring `dynamic_obstacle_predictor` against a recording
instead of judging it by eye in RViz. Built to answer one question
objectively: how often does it predict motion for something that never
moved?

## Ground truth

There are no labels in the bags, so truth comes from a **vacancy test** on
the raw cloud: a thing that genuinely moved has LEFT, so the ground it
occupied is empty a couple of seconds later. A fence is still exactly where
it was — however convincingly its cluster centroid appeared to slide as the
robot drove past it.

The test is anchored on a cluster's own measured points, not on any derived
position: a derived position wanders into the gaps of a big sprawling
cluster and reads as "vacated" while the structure is plainly still there.

## Usage

```bash
# 1. Once per source bag: derive a compact bag holding just the segmented
#    obstacle cloud + TF, so the predictor can be replayed on its own.
./tools/record_derived_bag.sh /home/adam/ws/rosbags/jk0009 \
                             /home/adam/ws/rosbags/derived_jk0009

# 2. Score a configuration (extra args are passed to the node as ROS params)
./tools/run_eval.sh mylabel 160 2.0 -p max_persistence:=0.3

# 3. Score against a different scene
EVAL_BAG=/home/adam/ws/rosbags/derived_pivo0001 ./tools/run_eval.sh mylabel 300 2.0
```

The headline number is `fp_frames`: how many predictor cycles a bogus
prediction was live, i.e. how much the costmap actually got polluted.
`tp_frames` is the same count for predictions on things that really did
move, and must not collapse — suppressing everything is not a fix.

Nothing else may be publishing the predictor's topics while an evaluation
runs; a stray node from an earlier run silently mixes into the results.
`run_eval.sh` clears them before and after, but check with `ps` if numbers
look impossible.

Results land in `tools/results/<label>.json` (git-ignored) and include a
per-track breakdown, so a threshold can be chosen from the measured
distribution rather than guessed.

## Measured

Both scenes, comparing the committed baseline against the footprint-vacancy
classifier (mean of repeated runs, replay rate 2.0):

| scene    | fp_frames        | tp_frames        | fp_rate       |
|----------|------------------|------------------|---------------|
| jk0009   | 2613 -> 211 (-92%) | 1143 -> 806 (-29%) | 0.696 -> 0.207 |
| pivo0001 | 1707 -> 111 (-93%) | 1035 -> 831 (-20%) | 0.623 -> 0.118 |

pivo0001 was not used for tuning.

### segment_max_points

A point count says how CLOSE a cluster is, not how big -- the Livox puts
400-1600 points on a person at 3-4 m. The cap of 600 was harmless while the
input was a LaserScan of a few hundred rays, but on the pointcloud it deleted
pedestrians outright: in jk0009 the person walking past at t=9-16 s produced a
raw segment in 6 frames out of ~70 and never a prediction. Raised to 5000
(a runaway guard; size is `segment_max_width`'s job):

| scene    | fp_frames | tp_frames  | fp_rate       | pedestrians found |
|----------|-----------|------------|---------------|-------------------|
| jk0009   | 207 -> 300 | 663 -> 1488 | 0.238 -> 0.168 | 10 -> 23 |
| pivo0001 | 105 -> 130 | 911 -> 1167 | 0.103 -> 0.100 | 15 -> 15 |

Predictions on long-lived scenery stay put (103 -> 104 frames on jk0009,
80 -> 98 on pivo0001), so the extra fp_frames are short-lived tracks, not a
return of the fence problem.
