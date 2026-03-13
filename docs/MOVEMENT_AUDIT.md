# Movement Audit — RoboticManipulationV2

**Date:** March 2026  
**Scope:** All movement, trajectory, and motion execution paths.

---

## 1. Executive Summary

| Area | Status | Notes |
|------|--------|------|
| **Primary stack** | ✅ | `HardwareInterface` + `moveToPose` / `movePosePath` is the main path; well-structured. |
| **Legacy / dual paths** | ⚠️ | `moveToPosition` (Common) and `HardwareSyncController` use different IK/units and reference missing helpers. |
| **Safety** | ✅ | Z-floor, joint limits, and reroute-via-safe-Z are implemented in the main stack. |
| **Angle convention** | ⚠️ | Radians (Common IK + `angleConversion`) vs degrees (`OpenManipulator.IK` + `HardwareInterface`) — document and avoid mixing. |
| **Trajectory** | ✅ | `smoothTrajectory` exists but is **unused**; task scripts use waypoint sequences and `movePosePath` instead. |

---

## 2. Movement Entry Points

### 2.1 Primary (recommended)

- **`OpenManipulator.HardwareInterface`** (`src/+OpenManipulator/HardwareInterface.m`)
  - **`moveToPose(x, y, z, pitch, time_sec, mode, z_floor_mm, ctx_or_fly)`**  
    Single pose move with mode (1=Joint, 2=Task linear, 3=Jacobian). Auto reroutes via safe Z when direct path would go below floor.
  - **`movePosePath(waypoints, opts)`**  
    N×4 `[x y z pitch]` path; continuous stream with `speed_mm_s`, `dt`, `z_floor_mm`, smoothing.
  - **`moveToAngles(q_deg)`**  
    Direct joint move (degrees), sync write + wait.
  - **`moveToAnglesInterpolated(q_target, num_steps, z_floor_mm)`**  
    Software-interpolated joint move with EE Z-floor (and legacy structure) safety.
  - **`executePoseMove(...)`**  
    Internal: one segment, no reroute; used by `moveToPose` and bridge context.

All use **degrees** and **`OpenManipulator.IK`** (returns degrees). Encoder conversion: **`deg2enc` / `enc2deg`** (class static).

### 2.2 Task / script layer

- **`move_seq`, `move_seq_pick`, `move_seq_pour`, `move_seq_cup2_contact`** (e.g. `scripts/task2d_cups_and_stir.m`)
  - Call either `hw.movePosePath(waypoints, opts)` (mode 2) or a loop of `hw.moveToPose(...)`.
  - Speeds vary by use (e.g. pour 23 mm/s, cup2_contact 15 mm/s).

### 2.3 Common (legacy / alternate)

- **`Common/moveToPosition.m`**
  - **Signature:** `moveToPosition(port_num, lib_name, target_pos, target_orientation, speed)`
  - Uses **`inverseKinematics`** (Common), which returns **radians**.
  - Uses **`angleConversion('rad2enc', q(i), i)`** with joint index (handles joint 2/3 sign).
  - Writes goal position per motor then waits on MOVING + position threshold.
  - **Not used** by current task scripts (they use `HardwareInterface`). Kept for scripts that pass `port_num`/`lib_name` directly.

- **`Common/HardwareSyncController.m`**
  - Polls a bridge server and calls **`inverseKinematicsAuto`** and **`moveJoint`**.
  - **`inverseKinematicsAuto`** and **`moveJoint`** are **not defined** in the repo (legacy/broken). Will error at runtime.

---

## 3. Trajectory and Interpolation

| Component | Role | Used? |
|-----------|------|--------|
| **`smoothTrajectory.m`** | Lift → horizontal → descend waypoints (linear segments) | ❌ Not referenced |
| **`movePosePath`** | Continuous N×4 waypoints; linear or smoothstep in `s`; IK per step | ✅ Yes (task2d, task3) |
| **`executePoseMove` (mode 2)** | Linear or smoothstep in `s`; pose interpolated; IK each step | ✅ Yes |
| **`moveToAnglesInterpolated`** | Linear in joint space; step-based; FK safety each step | ✅ Yes (mode 1 and segment end lock) |

So “movement” in practice is either:
- **Pose space:** interpolate (x,y,z,pitch) then IK (and optionally clamp) per step, or
- **Joint space:** interpolate q then FK check per step.

No use of `smoothTrajectory`; scripts build explicit waypoint lists instead.

---

## 4. Safety

### 4.1 Implemented

- **Z floor (`z_floor_mm`)**  
  - In **`moveToPose`**: simulated path checked (joint-space for mode 1, Cartesian for 2/3); if any point &lt; floor → reroute via safe Z (lift → move at safe Z → descend).
  - In **`executePoseMove` (mode 2)**: commanded pose Z checked each step; abort if &lt; `z_floor_mm`.
  - In **`movePosePath`**: commanded Z checked each step; **errors** if below floor.
  - In **`moveToAnglesInterpolated`**: optional EE Z-floor; can allow upward-only recovery when already below floor.

- **Joint limits**  
  - **`OpenManipulator.JointLimits`**: 4×2 [min, max] degrees; **Clamp(q)** used after IK in HardwareInterface (e.g. in `executePoseMove` mode 2, `movePosePath`).
  - HardwareInterface **configure()** writes **ADDR_MIN/MAX_POSITION_LIMIT** from `JointLimits.GetLimits()` (converted via `deg2enc`).

- **Tests**  
  - **`tests/test_safety_check.m`** simulates a path that dips below Z=20 and checks that the same logic (elbow/wrist/EE Z) would trigger.

### 4.2 Gaps / notes

- **`moveToPosition.m`** does **not** use `JointLimits` or a Z floor; it trusts Common IK (which has its own collision checks). Any script using it should ensure targets are safe.
- **HardwareSyncController** is broken (missing `inverseKinematicsAuto`, `moveJoint`) and not safety-audited.

---

## 5. Angle and Unit Conventions

| Code path | Angle unit | Encoder conversion |
|-----------|------------|--------------------|
| **HardwareInterface** (and `OpenManipulator.IK`, FK) | **Degrees** | `deg2enc` / `enc2deg` (no joint index; symmetric) |
| **Common `moveToPosition`** + **`inverseKinematics`** | **Radians** | `angleConversion('rad2enc', q, joint_idx)` (joint 2/3 sign and offset) |

So:

- **`OpenManipulator.IK`** → degrees → **HardwareInterface** is consistent.
- **`inverseKinematics` (Common)** → radians → **angleConversion** is consistent.
- **Mixing** the two (e.g. feeding Common IK output into HardwareInterface without rad→deg) would be wrong. No such mix was found in the audited call sites.

**Recommendation:** Document “degrees in HardwareInterface and OpenManipulator.*, radians in Common inverseKinematics and moveToPosition” in a single CONVENTIONS.md or in each file header, and avoid adding new code that mixes them.

---

## 6. Sync and Timing

- **HardwareInterface** uses **groupSyncWrite** for goal position; all 4 joints commanded in one packet.
- **Wait for motion:** `waitForMotion(timeout)` polls **ADDR_MOVING** until all stop, with short double-check (0.02 s) to reduce segment-end hitch.
- **moveToPosition (Common):** separate writes per motor (“pseudo-sync”), then wait on MOVING + position threshold (20 encoder units).
- **movePosePath / executePoseMove:** fixed **dt** (e.g. 0.02 s) between sync writes; optional tail (repeat last command a few times) then `waitForMotion()`.

---

## 7. Recommendations

1. **Prefer single stack:** Use **HardwareInterface** + **moveToPose** / **movePosePath** for all new tasks. Treat **moveToPosition** and **HardwareSyncController** as legacy.
2. **Fix or remove HardwareSyncController:** Either implement **`inverseKinematicsAuto`** and **`moveJoint`** (and align with one IK/angle convention), or remove/disable the script and document it as deprecated.
3. **Document angle conventions:** Add a short CONVENTIONS.md (or equivalent) stating: degrees in `OpenManipulator.*` and HardwareInterface; radians in Common `inverseKinematics` and `moveToPosition`.
4. **smoothTrajectory:** Either start using it (e.g. for pick/place lift–transit–descend) or remove it to avoid dead code.
5. **moveToPosition:** If kept, add optional Z-floor and joint-limit check (or a single “safe move” wrapper) so it matches the safety model of the main stack.

---

## 8. File Reference

| File | Purpose |
|------|---------|
| `src/+OpenManipulator/HardwareInterface.m` | Main hardware and movement API (sync write, moveToPose, movePosePath, safety). |
| `src/+OpenManipulator/IK.m` | IK in degrees for HardwareInterface. |
| `src/+OpenManipulator/JointLimits.m` | Limits in degrees; Clamp/Validate. |
| `Common/moveToPosition.m` | Legacy XYZ move (radians, angleConversion). |
| `Common/smoothTrajectory.m` | Unused lift–transit–descend waypoints. |
| `Common/inverseKinematics.m` | Radian IK; used by moveToPosition. |
| `Common/angleConversion.m` | rad/deg/enc with joint index for legacy path. |
| `Common/HardwareSyncController.m` | Bridge poll; references missing moveJoint / inverseKinematicsAuto. |
| `scripts/task2d_cups_and_stir.m` | move_seq*, moveToPose, movePosePath. |
| `tests/test_safety_check.m` | Safety logic test (Z floor style). |
