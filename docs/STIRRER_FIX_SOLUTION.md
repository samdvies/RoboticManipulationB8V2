# Stirrer Fix Solution

## Problem

The stirring motion in `scripts/task2d_cups_and_stir.m` was jerky and slow.

The root cause was that the stir loop executed as many small independent
`moveToPose(...)` calls:

- each stir point was treated as a separate motion segment
- each segment re-ran IK and motion execution independently
- each segment ended with tail commands, `waitForMotion()`, and verification
- the robot therefore stopped or nearly stopped at every stir waypoint

This created visible stop-start motion instead of a continuous circular stir.

## Previous Behavior

The old implementation:

- built a circular path from 16 discrete points
- repeated that path for 4 laps
- called `hw.moveToPose(...)` once per point

That meant 64 separate pose moves for the stir loop alone.

Even though `STIR_MOVE_TIME` was small, the lower-level executor still imposed
minimum duration and settle behavior per segment, so the overall motion felt
both slow and jerky.

## Implemented Fix

The stir loop was changed from segmented waypoint execution to a continuous
streamed Cartesian path.

### What changed

In `scripts/task2d_cups_and_stir.m`:

- the stir path is still defined as a circular set of points
- those points are now assembled into one continuous streamed path
- a new local helper, `stream_pose_path(...)`, sends interpolated IK-based
  commands continuously through the whole stir sequence

### New motion strategy

`stream_pose_path(...)`:

- starts from the robot's current actual pose
- interpolates smoothly from the current pose to each target stir waypoint
- uses cubic smoothstep interpolation:
  - `s_smooth = s * s * (3 - 2 * s)`
- solves IK at each streamed substep
- clamps joints with `OpenManipulator.JointLimits.Clamp`
- sends encoder targets directly with `hw.syncWritePositions(...)`
- avoids the repeated segment-end `moveToPose(...)` stop/wait cycle

At the end of the full streamed path, it:

- sends a short final tail of repeated target commands
- calls `hw.waitForMotion()` once

## Why this is better

This turns the stirring routine into one continuous command stream instead of a
sequence of independent stops.

Expected improvements:

- smoother circular motion
- less cornering jerk
- better apparent speed
- less visible hesitation between stir points

## Tunable Parameters

Current values in `scripts/task2d_cups_and_stir.m`:

- stir radius: `7.5 mm`
- stir points per lap: `16`
- laps: `4`
- stream speed: `120 mm/s`
- stream timestep: `0.02 s`

If further tuning is needed:

- increase `stir_path_points` for a rounder path
- reduce stream speed if the robot overshoots
- increase stream speed if the motion is still too slow
- reduce radius if the stirrer is contacting the cup wall

## Files Changed

- `scripts/task2d_cups_and_stir.m`

## Additional Motion Tweaks

After the continuous stir fix, two more motion-geometry changes were applied in
the same script:

### 1. Higher stirrer clearance before and after pickup

The stirrer hover height was increased:

- from `210 mm`
- to `240 mm`

This affects:

- the pre-pick hover above the stirrer
- the immediate post-pick lift away from the pickup area
- the return/release clearance path

The purpose is to reduce collision risk around the stirrer pickup zone.

### 2. Final pour arc moved inward

The final pour "mouth" arc was shifted `50 mm` closer to the robot base along
the radial direction from the origin.

This affects:

- `mouth_start`
- `mouth_mid`
- `mouth_end`

The Z heights and pitch values for those arc poses were left unchanged.

## Summary

The fix does not change the high-level stirring shape. It changes how the
trajectory is executed:

- before: many discrete `moveToPose(...)` calls
- after: one continuous streamed pose path

That is the key change that removes the stop-start behavior.
