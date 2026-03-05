# Cube Reorientation and Stack Offset Compensation (MATLAB-First Design)

## Goal
Enable reliable stacking of cubes with the red side up, using a deterministic pick-reorient-place flow, despite off-center grasp caused by holder lip geometry.

Primary execution target is MATLAB scripts/hardware flow, not the Python simulation.

## Problem Summary
The current reorientation primitive (pick at roughly `0 deg`, place/reorient at `-90 deg`) is functionally good, but holder lip contact means the cube is not always grasped perfectly centered in the jaws.

When a non-centered grasp is rotated, the cube center shifts in XY. For stacking, this creates cumulative placement bias and potential instability.

## Why This Approach
No vision feedback is available, so dynamic correction is not possible.

Given constraints, the most robust option is:
1. Make the manipulation primitive deterministic.
2. Calibrate systematic XY offset once for the holder type.
3. Apply compensation at placement time.

This converts repeatable geometric bias into a fixed correction term.

## Key Assumptions
1. Holder type is fixed and always oriented the same way (facing robot base).
2. Cube and gripper interaction is repeatable enough that bias is mostly systematic.
3. Pickup height, jaw closure command, and reorientation trajectory are held constant between calibration and production.
4. Random motor/drive error is around `<= 3 mm` (as observed), so systematic component should be reduced below that.

## Non-Goals
1. No camera/perception integration.
2. No online estimation of cube pose.
3. No change to bridge avoidance logic for this feature.
4. No requirement to modify Python solver behavior.

## Core Design
Use a fixed compensation vector for this holder type:
- `dx_mm`: measured X bias after reorientation
- `dy_mm`: measured Y bias after reorientation

For a desired stack center `(x_stack, y_stack)`, command:
- `x_cmd = x_stack - dx_mm`
- `y_cmd = y_stack - dy_mm`

`z` and orientation logic remain unchanged.

## Calibration Procedure (Offline, MATLAB)
Create a repeatable script-driven process:

1. Use fixed pickup pose and jaw command.
2. Execute the exact production reorientation primitive (`0 -> -90`) each trial.
3. Place onto a known reference XY target on a flat surface.
4. Measure resulting XY miss per trial (manual measurement is acceptable).
5. Repeat for `N=10` to `N=20`.
6. Compute:
   - `dx_mm = mean(x_error_mm)`
   - `dy_mm = mean(y_error_mm)`
   - `sigma_x`, `sigma_y` for drift awareness.
7. Save to calibration artifact (MAT-file).

Suggested file:
- `scripts/calibration/reorient_offset.mat`

Suggested structure:
```matlab
reorientOffset.dx_mm
reorientOffset.dy_mm
reorientOffset.sigma_x_mm
reorientOffset.sigma_y_mm
reorientOffset.n_trials
reorientOffset.pickup_z_mm
reorientOffset.jaw_cmd
reorientOffset.timestamp
```

## MATLAB Implementation Plan

### 1) New helper: apply compensation
Add `Common/applyReorientOffset.m`:
```matlab
function [x_cmd, y_cmd] = applyReorientOffset(x_stack, y_stack, offset)
    x_cmd = x_stack - offset.dx_mm;
    y_cmd = y_stack - offset.dy_mm;
end
```

### 2) New calibration script
Add `scripts/calibrate_reorient_offset.m`.
Responsibilities:
- Run calibration trials.
- Log measured XY errors.
- Compute means/std.
- Save MAT artifact.

### 3) Integrate into stack/place flows
Likely touchpoints:
- `scripts/test_pick_and_place.m`
- `scripts/bridge_pick.m`
- `scripts/test_bridge_pick.m` (if this path is used for final place)

At final place stage:
1. Load calibration struct.
2. Compute compensated `x_cmd, y_cmd`.
3. Execute place move using compensated target.

### 4) Validation script
Add `scripts/validate_reorient_offset.m`.
Run 20-cycle test and report:
- mean XY error
- max XY error
- percentage within ±3 mm

Suggested pass criteria:
- mean error < 1.5 mm per axis
- >=95% placements within ±3 mm

## Operational Rules
1. Recalibrate if any of these change:
   - holder geometry
n   - pickup Z
   - jaw close setpoint
   - reorientation timing/profile
2. If calibration file missing, either:
   - block stack run, or
   - continue in explicit "uncompensated mode" with warning.

## Risks and Mitigations

### Risk 1: Bias not fully systematic
If friction/slip randomness dominates, fixed offset won’t fully fix stack alignment.

Mitigation:
- Increase trial count.
- Tighten grasp consistency (jaw command, approach speed, dwell).
- Use conservative stack height growth until consistency is validated.

### Risk 2: Drift over time
Hardware wear or slight fixture movement can shift bias.

Mitigation:
- Periodic quick validation run.
- Recalibrate when validation threshold fails.

### Risk 3: Under-bridge place path interaction
Route is safe but may slightly change end dynamics.

Mitigation:
- Keep final descent/placement primitive identical regardless of prior route.
- Apply compensation only at final place target computation.

## Decision Log
1. Use fixed offset compensation instead of perception.
   - Reason: no camera feedback, systematic bias expected.
2. Calibrate by holder type/orientation, not world location.
   - Reason: holder orientation is fixed; translation should not change local bias.
3. Preserve current manipulation and safety logic.
   - Reason: movement currently stable and bridge behavior improved.

## Minimal Checklist (When You Implement Later)
1. Add `applyReorientOffset.m`.
2. Add calibration script and MAT save/load.
3. Hook compensation into final place target.
4. Add validation script and thresholds.
5. Document recalibration trigger conditions.

## Notes for Python Simulation
This feature is mostly hardware-facing. Python sim can optionally mirror the compensation variable for workflow parity, but should not be considered ground truth for calibration values.
