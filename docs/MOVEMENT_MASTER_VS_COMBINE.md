# Movement implementation: master vs current (combine) branch

Pick and place works perfectly on **master**. This doc summarizes how movement differs on the **combine** branch so you can relate behavior or regressions to specific changes.

**Scope:** All movement is driven by `OpenManipulator.HardwareInterface` in `src/+OpenManipulator/HardwareInterface.m`. That is the **only file** whose movement implementation differs between branches. Scripts and `Common/` use the same `moveToPose` / `moveToAnglesInterpolated` API; only the internals of `moveToPose` and `executePoseMove` change.

---

## 1. High-level summary

| Aspect | Master | Combine |
|--------|--------|--------|
| **9th argument to moveToPose** | `bridge_ctx` (struct or []) | `ctx_or_fly` (struct **or** logical: “fly” = higher Cartesian speed) |
| **Control loop rate (non-bridge)** | 20 Hz (`dt = 0.05`) | 50 Hz (`dt = 0.02`) |
| **Segment duration (non-bridge)** | Fixed: `max(time_sec, 0.1)` for linear part | **Distance-scaled**: duration from Cartesian speed (70 or 180 mm/s), then capped by `time_sec` |
| **Interpolation (non-bridge mode 2)** | C1 smooth: `s_smooth = s²(3−2s)` | **Linear**: `s` (no smoothing) |
| **Mode 2 end-of-segment** | `waitForMotion` → lock (elbow_up) → `verifyPose` | **Bridge:** same but lock uses **elbow_down**. **Non-bridge:** 5-step “tail” (resend target at 50 Hz), then wait, then optional VERIFY log, then `verifyPose` |
| **Safety check before move** | Joint-space sim only (linear blend q_current → q_target) | **Mode 2/3:** Cartesian line (start → target pose) for Z check. **Mode 1:** same joint-space sim as master |
| **Reroute (unsafe direct path)** | Via points get same `bridge_ctx` | Final “lower to target” segment gets **`false`** (no context) |
| **waitForMotion** | Poll at 0.05 s; 0.1 s “double-check” pause when stopped | Poll at **0.02 s**; **0.02 s** double-check (reduced to limit segment-end hitch) |
| **VERIFY logging** | None | **Non-bridge:** VERIFY START / during segment / VERIFY END (mode 1, 2, 3) when `log_verify_pose` is true |
| **Skip tiny moves** | No | **Yes:** if `dist_lin < 1e-6` and `dist_rot < 0.5°`, return immediately |
| **Jacobian (mode 3)** | `J_pitch_row = [0, 1, 1, 1]`; `vel_mag` from `time_sec` | **J_pitch_row = [0, -1, -1, -1]** (pitch = -(q2+q3+q4)); `vel_mag` from **duration**; final phase has 5-step tail + optional VERIFY like mode 2 |

---

## 2. moveToPose: 9th argument and “fly”

- **Master:**  
  `moveToPose(..., z_floor_mm, bridge_ctx)`  
  - `bridge_ctx` = struct (with `.preplanned_route`, `.zones`, etc.) or `[]`.  
  - Used only for bridge/preplanned behavior and optional dynamic pitch.

- **Combine:**  
  `moveToPose(..., z_floor_mm, ctx_or_fly)`  
  - **Struct** with `.preplanned_route` etc. → same as master (bridge context); when `preplanned_route == true`, combine uses **“bridge_pick timing”** (same dt, duration rule, and C1 smoothing as master).  
  - **Logical** → `is_fly`: when true, non-bridge segment uses higher Cartesian speed (180 mm/s, max 300 mm/s) instead of normal (70 mm/s, max 120 mm/s).  
  - Default when 9th arg omitted: `false` (not fly, no bridge context).

So on combine, **pick/place that never passes a 9th argument** is unchanged in terms of “bridge vs not”; it gets the new default `false` and the new **non-bridge** timing and interpolation.

---

## 3. executePoseMove: timing and interpolation

### 3.1 Control rate and “bridge_pick timing”

- **Master:**  
  Single rate: `dt = 0.05` (20 Hz) for all segments.

- **Combine:**  
  - If **bridge context** with `preplanned_route == true`:  
    `use_bridge_pick_timing = true` → `dt = 0.05`, no VERIFY logging, duration and interpolation same as master (see below).  
  - Otherwise (normal pick/place, fly, etc.):  
    `dt = 0.02` (50 Hz), VERIFY logging on (when enabled), **distance-scaled duration** and **linear** interpolation in mode 2.

So **normal pick/place on combine** runs at 50 Hz and uses the new duration and interpolation rules.

### 3.2 Duration (non-bridge)

- **Master:**  
  - `dur_lin = max(time_sec, 0.1)` if `dist_lin > 0`, else 0.  
  - `duration = max(dur_lin, dur_rot, 0.1)`, `num_steps = ceil(duration / dt)`.

- **Combine (when not use_bridge_pick_timing):**  
  - Linear duration from Cartesian speed:  
    - Normal: `cartesian_speed_mm_s = 70`, `max_cartesian_speed_mm_s = 120`.  
    - Fly: 180 and 300.  
  - `dur_lin = dist_lin / cartesian_speed_mm_s`, clamped to at least 0.15 s and at most `time_sec`, and not faster than max speed.  
  - `duration = max(dur_lin, dur_rot, 0.15)`, `num_steps = ceil(duration / dt)`.

So on combine, **short moves can be shorter in time** (higher effective speed) and **long moves are capped by `time_sec`**; on master, every move with linear distance effectively used at least `time_sec` (or 0.1 s).

### 3.3 Interpolation (mode 2, non-bridge)

- **Master:**  
  C1 smooth: `s_smooth = s * s * (3.0 - 2.0 * s)`, pose and pitch interpolated with `s_smooth`.

- **Combine (non-bridge):**  
  Linear: pose and pitch interpolated with `s` (no smoothing).  
  Bridge segments still use `s_smooth` when `use_bridge_pick_timing` is true.

So **pick/place on combine (mode 2)** has linear interpolation; start/end of segment can feel slightly different from master.

---

## 4. Mode 2: end-of-segment behavior

- **Master:**  
  After the step loop: `waitForMotion()` → try lock to target with IK(..., 'elbow_up', false) and one-step interpolated move → `verifyPose(q_target)`.

- **Combine:**  
  - **Bridge (`use_bridge_pick_timing`):**  
    Same idea, but lock uses IK(..., **'elbow_down'**) (comment: “Master: no tail; wait then end-of-segment lock then verify”).  
  - **Non-bridge:**  
    Send target pose for 5 steps at `dt` (small “tail”), then `waitForMotion()`, then optional VERIFY END log, then `verifyPose(q_target)`.

So for **non-bridge** mode 2, combine adds a short tail and (when enabled) VERIFY logging; for **bridge**, combine matches master flow but uses **elbow_down** for the lock.

---

## 5. Safety: “is direct path safe?” (before executing)

- **Master:**  
  Always checks a **joint-space** path: 10 steps from `q_current` to `q_target`, FK each step, flag unsafe if any EE Z &lt; z_floor.

- **Combine:**  
  - **Mode 2 or 3:**  
    Checks a **Cartesian line** from current EE position to target (x,y,z): 10 steps along that line, flag unsafe if any Z &lt; z_floor. So mode 2/3 no longer trigger reroute from a low joint-space arc that stays above floor in Cartesian space.  
  - **Mode 1:**  
    Same joint-space simulation as master.

So **only mode 1** keeps master’s safety logic; mode 2/3 use a Cartesian-line check.

---

## 6. Reroute (when direct path is unsafe)

- **Master:**  
  Three segments: lift at current XY → move at safe Z to target XY → lower to target. All three call `executePoseMove(..., bridge_ctx)` (same context as original call, or empty).

- **Combine:**  
  Same three segments, but the **last** segment (lower to target) is called with **`false`** as 9th argument:  
  `executePoseMove(q_target, [x,y,z], pitch, time_sec, mode, z_floor_mm, false)`.  
  So the final drop uses non-bridge timing and no context (no fly).

---

## 7. waitForMotion

- **Master:**  
  Poll “moving” at 0.05 s intervals; when all stopped, **pause(0.1)** then re-check.

- **Combine:**  
  Poll at **0.02 s**; when all stopped, **pause(0.02)** then re-check.  
  Comment: shorten pause to avoid a long gap with no position commands at segment end (reduces visible jump into next segment).

---

## 8. Mode 1 (joint interpolation)

- **Master:**  
  `moveToAnglesInterpolated(q_target, num_steps, z_floor_mm)` only; no logging.

- **Combine:**  
  Same call, but if `log_verify_pose` is true, after the move it reads angles, does FK, and prints VERIFY END (actual q, FK position, pitch, target, err_mm).  
  So **mode 1** is the same except optional VERIFY logging and the fact that **num_steps** (hence duration) can differ because of the new duration rule when mode was chosen by a previous layer (here all mode-1 segments get their num_steps from the same executePoseMove duration logic).

---

## 9. Mode 3 (Jacobian)

- **Master:**  
  - `J_pitch_row = [0, 1, 1, 1]` (pitch rate from joint rates).  
  - `vel_mag = max(dist_lin / max(time_sec, 0.1), 10.0)`.  
  - When switching to final joint interpolation phase: single `moveToAnglesInterpolated(new_q, 1, z_floor_mm)` then `verifyPose`.

- **Combine:**  
  - **J_pitch_row = [0, -1, -1, -1]** so that pitch convention matches `pitch_deg = -(q2+q3+q4)`.  
  - `vel_mag = max(dist_lin / max(duration, 0.1), 10.0)` (uses computed **duration** instead of `time_sec`).  
  - Final phase: 5-step tail (resend final target at dt), then `waitForMotion()`, then optional VERIFY END log, then `verifyPose`.  
  - Optional VERIFY logging during the loop (like mode 2).

So mode 3 has a **sign fix** for pitch, **duration-based** velocity scaling, and the same “tail + wait + verify” pattern as non-bridge mode 2.

---

## 10. What stays the same

- **moveToAngles**, **moveToAnglesInterpolated**, **readAngles**, **syncWritePositions**, **configure**, **enableTorque**, gripper, **verifyPose** implementation (except call sites and any logging around them).  
- **IK / FK / JointLimits.Clamp** usage and semantics.  
- **BridgeAvoidance** (only a BOM difference in the file).  
- When **bridge context with preplanned_route** is passed, combine explicitly uses master-equivalent timing, interpolation, and end-of-segment behavior for that segment.

---

## 11. Practical impact for “pick and place worked on master”

- **If your master pick/place uses only mode 1:**  
  Combine changes: (1) 50 Hz instead of 20 Hz, (2) distance-scaled duration (shorter moves can be faster), (3) shorter waitForMotion pause, (4) optional VERIFY logs. The **elbow lock** at end of mode 2 does not apply to mode 1.

- **If your master pick/place uses mode 2:**  
  Combine adds: (1) 50 Hz, (2) **linear** interpolation instead of C1, (3) **distance-scaled** duration, (4) 5-step tail at end of segment, (5) shorter waitForMotion, (6) VERIFY logging. So **timing and curvature** of each segment differ; any sensitivity to acceleration or segment timing could show up here.

- **If you ever hit “unsafe path” reroute on master:**  
  On combine, mode 2/3 use a **Cartesian** safety check, so fewer cases trigger reroute; when they do, the final “lower to target” segment runs with **non-bridge** timing and no context (could feel different from master’s three segments all using the same context).

- **Bridge picks:**  
  When scripts pass the bridge context struct with `preplanned_route == true`, combine uses the same timing and C1 interpolation as master for those segments (see BRIDGE_PICK_AUDIT.md).

Use this overview together with BRIDGE_PICK_AUDIT.md to trace any regressions (e.g. “pick/place feels different” or “one segment is faster/slower”) to a specific change above.
