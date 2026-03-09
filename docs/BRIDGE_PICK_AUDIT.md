# Bridge pick compatibility audit (master vs combine)

**Question:** If `bridge_pick.m` works on the **master** branch, will bridge picking work on the **current (combine)** branch?

**Conclusion: Yes.** The combine branch preserves master-equivalent behavior for bridge picks via explicit `use_bridge_pick_timing` in `HardwareInterface.m`. Both the standalone `bridge_pick.m` script and the task-driven `bridgePickCube` path use the same context and get the same timing and smoothing as on master.

---

## 1. Scope: files involved

| File | Role |
|------|------|
| `scripts/bridge_pick.m` | Standalone bridge-pick hardware script (master has it; on combine it is **deleted** in git; you have an **untracked** local copy). |
| `Common/bridgePickCube.m` | Task-flow bridge pick (combine only). Port of bridge_pick logic; same planning and same `moveToPose(..., ctx)` usage. |
| `src/+OpenManipulator/HardwareInterface.m` | Executes moves; 9th arg renamed `bridge_ctx` → `ctx_or_fly`; when `preplanned_route==true` it uses master timing. |
| `src/+OpenManipulator/BridgeAvoidance.m` | Bridge zones and waypoint planning. Effectively unchanged (only a BOM added). |
| `scripts/task_general_planner.m` | Emits `bridge_pick` steps and calls `bridgePickCube(hw, step.cube_xy, cfg)`. |

Other dependencies used by bridge picking: `OpenManipulator.IK`, `OpenManipulator.FK`, `OpenManipulator.JointLimits.Clamp` — no relevant changes on combine for this flow.

---

## 2. `scripts/bridge_pick.m`

- **Master:** Tracked; script runs and calls `moveToPose(..., Z_FLOOR, entry_ctx)` (and same for lift/exit) with structs that have `preplanned_route: true`.
- **Combine:** The file is **deleted** in the repo; your workspace has an **untracked** copy with the **same content** as master.
- When you run that script on combine, it still passes the same 9th argument (e.g. `entry_ctx`, `lift_ctx`, `exit_ctx`) with the same fields: `zones`, `final_target_pose`, `preplanned_route`, `dynamic_pitch`.

So the **call pattern** of `bridge_pick.m` is unchanged; only the implementation of `moveToPose` / `executePoseMove` on combine is different, and that implementation explicitly preserves bridge-pick behavior when it sees `preplanned_route == true`.

---

## 3. `HardwareInterface.m` (combine vs master)

### 3.1 Signature and meaning of the 9th argument

- **Master:** `moveToPose(..., z_floor_mm, bridge_ctx)` with `bridge_ctx` a struct or empty.
- **Combine:** `moveToPose(..., z_floor_mm, ctx_or_fly)` where:
  - struct with `.preplanned_route` etc. → treated as bridge context (same as master),
  - logical → `is_fly` for higher Cartesian speed (new use).

Bridge scripts only pass structs, so the **bridge-pick API** is backward compatible.

### 3.2 Detection of bridge-pick context

In `executePoseMove`, combine sets:

```matlab
use_bridge_pick_timing = isstruct(ctx_or_fly) && isfield(ctx_or_fly, 'preplanned_route') && ctx_or_fly.preplanned_route;
```

So whenever `bridge_pick.m` or `bridgePickCube.m` passes `entry_ctx` / `lift_ctx` / `exit_ctx` (all with `preplanned_route: true`), `use_bridge_pick_timing` is **true**.

### 3.3 Behavior when `use_bridge_pick_timing` is true

- **Timestep:** `dt = 0.05` (same as master).
- **Duration:** Same rule as master: `dur_lin = max(time_sec, 0.1)` when `dist_lin > 1e-6`, then `duration = max([dur_lin, dur_rot, 0.1])`, `num_steps = ceil(duration / dt)`.
- **Interpolation:** C1 smooth scaling `s_smooth = s*s*(3 - 2*s)` for pose and pitch (same as master).
- **End of segment:** Same as master: `waitForMotion()`, then IK lock with `elbow_down`, then `verifyPose(q_target)`; no extra “combine” tail or VERIFY logging that could change timing.

So for every move that is part of a preplanned bridge route, execution is **master-equivalent**.

### 3.4 Non-bridge moves (e.g. Phase 0 HOME)

- `bridge_pick.m` Phase 0 calls `moveToPose(home_pose(1), ..., MOVE_TIME, 1, Z_FLOOR)` with **8 arguments** (no 9th).
- Combine defaults the 9th to `false`, so no bridge context and no fly; normal combine behavior applies. That segment is not part of the bridge waypoint list, so differing timing there does not affect the conclusion that **bridge picking** matches master.

### 3.5 Safety reroute path

- When `preplanned_route` is true, `moveToPose` calls `executePoseMove` directly and does **not** run the safe-z reroute logic. Bridge waypoints are assumed already safe.
- So the reroute change on combine (e.g. last via segment passing `false` instead of context) does not affect bridge picks.

---

## 4. `BridgeAvoidance.m`

- **Diff vs master:** Only addition is a UTF-8 BOM at the start of the file (`﻿` before `classdef`). Behavior of `NewZone`, `BuildBridgeZones`, `PlanBridgeSafeWaypoints`, and `SolveOptimalPitch` is unchanged.
- Optional: remove the BOM to avoid any possibility of “invalid character” in strict environments; not required for correctness.

---

## 5. `Common/bridgePickCube.m` (combine only)

- Uses the same bridge geometry and planner options as `bridge_pick.m`.
- Builds the same context shape: `entry_ctx`, `lift_ctx`, `exit_ctx` with `zones`, `final_target_pose`, `preplanned_route: true`, `dynamic_pitch: false`.
- Calls `hw.moveToPose(..., MOVE_TIME, exec_mode_wp, Z_FLOOR, entry_ctx)` (and similarly for lift/exit). So it goes through the same `use_bridge_pick_timing` path and gets master-equivalent timing and smoothing.

So **task-driven** bridge picks (via `task_general_planner` → `bridge_pick` → `bridgePickCube`) are consistent with standalone `bridge_pick.m` and with master.

---

## 6. `task_general_planner.m`

- Emits steps with `action == 'bridge_pick'` and calls `bridgePickCube(hw, step.cube_xy, cfg)`.
- No change needed for this audit; the bridge-pick execution contract is fulfilled by `HardwareInterface` and `bridgePickCube` as above.

---

## 7. Summary table

| Check | Result |
|-------|--------|
| Same bridge context struct shape from script / bridgePickCube | Yes |
| `moveToPose` / `executePoseMove` accept it and set `use_bridge_pick_timing` | Yes |
| When true: same dt, duration rule, C1 smoothing, end-of-segment behavior as master | Yes |
| BridgeAvoidance logic unchanged | Yes (BOM only) |
| 8-arg moveToPose (e.g. Phase 0) still valid | Yes (9th defaults to false) |
| Task-driven bridge picks use same path | Yes (bridgePickCube → same ctx) |

---

## 8. Recommendation

- **If `bridge_pick.m` works on master**, running your (untracked) `scripts/bridge_pick.m` on the **combine** branch will use the same bridge execution path and preserve master-equivalent behavior for all entry/lift/exit waypoint moves.
- **Bridge picks from the task flow** (via `bridgePickCube`) also use that path, so they will behave the same as the standalone script for those segments.
- Optional: remove the BOM from `src/+OpenManipulator/BridgeAvoidance.m` for cleanliness; it does not affect this conclusion.

**Audit date:** 2025-03-09.  
**Branches:** master (reference), combine (current).
