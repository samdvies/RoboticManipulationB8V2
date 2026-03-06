#!/usr/bin/env python3
"""
Task 2.c — Tool Through Gates (Constant-Z XY Motion)

Generates and simulates a pick-and-place path that moves a tool through
a series of gates.  During XY traversal segments the commanded Z is
held perfectly constant; Z changes only during explicit lift/lower phases.

Standalone — prints/logs results.  No GUI required.

Uses:
    visualization/kinematics/IK.py   →  IK(x, y, z, pitch)
    visualization/kinematics/FK.py   →  FK(q)
    visualization/kinematics/JointLimits.py  →  clamp_joints, validate_joints
"""

import sys, os, math
import numpy as np

# Allow standalone execution from any directory
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_DIR = os.path.dirname(_THIS_DIR)
if _REPO_DIR not in sys.path:
    sys.path.insert(0, _REPO_DIR)

from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.kinematics.JointLimits import clamp_joints, validate_joints

# ── Parameters ────────────────────────────────────────────────────────
# All coordinates in mm, angles in degrees.
#
# WORKSPACE CONSTRAINTS (OpenManipulator-X):
#   Joint limits: J1 ±90°, J2 ±117°, J3 ±117°, J4 ±117°
#   Max reach:    ~380 mm (L2+L3+L4 = 130+124+126)
#   Pitch = -90° (vertical tool) is feasible at Z ≤ ~80mm for typical XY.
#   SAFE_Z is kept at 80mm so the tool stays vertical throughout.

TOOL_PICK_XY   = (-50.0, -200.0)   # Tool pickup location (XY)
TOOL_GRASP_Z   = 50.0              # Z to lower to for grasping
SAFE_Z         = 75.0              # Travel height (just above gate Z, keeps pitch=-90 feasible)
GATE_Z         = 70.0              # Z while passing through gates

# Gate waypoints — move through gates in order
GATE_WAYPOINTS_XY = [
    (100.0, -175.0),   # Waypoint 1
    (100.0,  -75.0),   # Waypoint 2
    (225.0,   65.0),   # Waypoint 3 (Gate entry)
    (225.0,   35.0),   # Waypoint 4 (Gate exit)
    (175.0,    0.0),   # Waypoint 5
    (175.0,  100.0),   # Waypoint 6
]

DROP_XY        = (50.0, 100.0)     # Drop-off location (XY)
DROP_Z         = 50.0              # Z for releasing
PITCH_DEG      = -85.0             # EE pitch for grasping / gate traversal (tilted to reach further)
TRAVEL_PITCH   = -85.0             # EE pitch during travel
STEP_MM        = 5.0               # Interpolation resolution


# ── Motion Generators ─────────────────────────────────────────────────

def _interpolate_axis(start, end, fixed, z, pitch, axis, step_mm):
    """Interpolate a single axis while holding the other axis and Z fixed."""
    if axis == "x":
        if abs(end - start) < 1e-9:
            return [[start, fixed, z, pitch]]
        n_steps = max(1, int(math.ceil(abs(end - start) / step_mm)))
        return [[start + (end - start) * (i / n_steps), fixed, z, pitch] for i in range(n_steps + 1)]

    if abs(end - start) < 1e-9:
        return [[fixed, start, z, pitch]]
    n_steps = max(1, int(math.ceil(abs(end - start) / step_mm)))
    return [[fixed, start + (end - start) * (i / n_steps), z, pitch] for i in range(n_steps + 1)]


def interpolate_xy_constant_z(start_pose, end_pose, step_mm=STEP_MM, axis_order="x_then_y"):
    """Interpolate XY with Manhattan motion while holding Z and pitch constant.

    Args:
        start_pose: [x, y, z, pitch]
        end_pose:   [x, y, z, pitch] (z and pitch are ignored; start values are used)
        step_mm:    max XY distance per step
        axis_order: "x_then_y" or "y_then_x"

    Returns:
        List of [x, y, z, pitch] poses.
    """
    sx, sy, sz, sp = start_pose
    ex, ey, _, _ = end_pose

    if abs(ex - sx) < 1e-9 and abs(ey - sy) < 1e-9:
        return [list(start_pose)]

    if axis_order not in ("x_then_y", "y_then_x"):
        raise ValueError(f"Invalid axis_order: {axis_order}")

    if axis_order == "x_then_y":
        seg1 = _interpolate_axis(sx, ex, sy, sz, sp, "x", step_mm)
        seg2 = _interpolate_axis(sy, ey, ex, sz, sp, "y", step_mm)
    else:
        seg1 = _interpolate_axis(sy, ey, sx, sz, sp, "y", step_mm)
        seg2 = _interpolate_axis(sx, ex, ey, sz, sp, "x", step_mm)

    poses = seg1 + seg2[1:]
    return poses


def interpolate_z_only(pose, target_z, step_mm=STEP_MM):
    """Change Z while keeping X, Y, and pitch fixed.

    Args:
        pose:     [x, y, z, pitch]
        target_z: desired Z
        step_mm:  max Z distance per step

    Returns:
        List of [x, y, z, pitch] poses.
    """
    x, y, z, p = pose
    dz = target_z - z

    if abs(dz) < 1e-9:
        return [list(pose)]

    n_steps = max(1, int(math.ceil(abs(dz) / step_mm)))
    poses = []
    for i in range(n_steps + 1):
        t = i / n_steps
        poses.append([x, y, z + dz * t, p])
    return poses


def interpolate_z_and_pitch(pose, target_z, target_pitch, step_mm=STEP_MM):
    """Change Z and pitch simultaneously while keeping X, Y fixed.

    Args:
        pose:         [x, y, z, pitch]
        target_z:     desired Z
        target_pitch: desired pitch at end
        step_mm:      max Z distance per step

    Returns:
        List of [x, y, z, pitch] poses.
    """
    x, y, z, p = pose
    dz = target_z - z
    dp = target_pitch - p

    dist = max(abs(dz), 1e-9)
    n_steps = max(1, int(math.ceil(dist / step_mm)))
    poses = []
    for i in range(n_steps + 1):
        t = i / n_steps
        poses.append([x, y, z + dz * t, p + dp * t])
    return poses


# ── Path Builder ──────────────────────────────────────────────────────

def build_task2c_path(
    tool_xy=TOOL_PICK_XY,
    grasp_z=TOOL_GRASP_Z,
    safe_z=SAFE_Z,
    gate_z=GATE_Z,
    gate_wps=GATE_WAYPOINTS_XY,
    drop_xy=DROP_XY,
    drop_z=DROP_Z,
    pitch=PITCH_DEG,
    travel_pitch=TRAVEL_PITCH,
    step_mm=STEP_MM,
):
    """Build the complete Task 2.c pose list.

    Uses `travel_pitch` during high-Z XY travel to keep q4 within the
    ±117° wrist limit.  Transitions pitch smoothly during lift/lower.

    Returns:
        poses:    list of [x, y, z, pitch]
        labels:   parallel list of phase labels (str) for logging
    """
    poses  = []
    labels = []

    def _add(segment_poses, label):
        poses.extend(segment_poses)
        labels.extend([label] * len(segment_poses))

    # ── 1. Approach above tool at SAFE_Z with travel pitch ──
    start_above = [tool_xy[0], tool_xy[1], safe_z, travel_pitch]
    _add([start_above], "approach_above_tool")

    # ── 2. Lower to grasp Z, transitioning pitch from travel → work ──
    _add(interpolate_z_and_pitch(start_above, grasp_z, pitch, step_mm), "lower_to_grasp")

    grasp_pose = [tool_xy[0], tool_xy[1], grasp_z, pitch]

    # ── 3. Close gripper (marker) ──
    _add([list(grasp_pose)], "close_gripper")

    # ── 4. Lift to SAFE_Z, transitioning pitch from work → travel ──
    _add(interpolate_z_and_pitch(grasp_pose, safe_z, travel_pitch, step_mm), "lift_to_safe_z")

    # ── 5. XY move at SAFE_Z to first gate entry (travel pitch) ──
    safe_pose  = [tool_xy[0], tool_xy[1], safe_z, travel_pitch]
    gate_entry = [gate_wps[0][0], gate_wps[0][1], safe_z, travel_pitch]
    _add(interpolate_xy_constant_z(safe_pose, gate_entry, step_mm), "xy_to_gate_entry")

    # ── 6. Lower to GATE_Z, transitioning pitch from travel → work ──
    at_entry_safe = [gate_wps[0][0], gate_wps[0][1], safe_z, travel_pitch]
    _add(interpolate_z_and_pitch(at_entry_safe, gate_z, pitch, step_mm), "lower_to_gate_z")

    # ── 7. Move through gate waypoints at constant GATE_Z + work pitch ──
    for i in range(len(gate_wps) - 1):
        wp_from = [gate_wps[i][0],   gate_wps[i][1],   gate_z, pitch]
        wp_to   = [gate_wps[i+1][0], gate_wps[i+1][1], gate_z, pitch]
        _add(interpolate_xy_constant_z(wp_from, wp_to, step_mm), f"gate_segment_{i+1}")

    # ── 8. Lift after gates, transitioning pitch from work → travel ──
    last_wp = [gate_wps[-1][0], gate_wps[-1][1], gate_z, pitch]
    _add(interpolate_z_and_pitch(last_wp, safe_z, travel_pitch, step_mm), "lift_after_gates")

    # ── 9. XY move at SAFE_Z to drop location (travel pitch) ──
    from_safe = [gate_wps[-1][0], gate_wps[-1][1], safe_z, travel_pitch]
    to_drop   = [drop_xy[0], drop_xy[1], safe_z, travel_pitch]
    _add(interpolate_xy_constant_z(from_safe, to_drop, step_mm), "xy_to_drop")

    # ── 10. Lower to DROP_Z, transition pitch → work, and release ──
    above_drop = [drop_xy[0], drop_xy[1], safe_z, travel_pitch]
    _add(interpolate_z_and_pitch(above_drop, drop_z, pitch, step_mm), "lower_to_drop")

    drop_pose = [drop_xy[0], drop_xy[1], drop_z, pitch]
    _add([list(drop_pose)], "open_gripper")

    return poses, labels


# ── Simulator ─────────────────────────────────────────────────────────

def simulate_path(poses, labels=None):
    """Run IK → FK for each pose, log results, and verify Z-constancy.

    Returns:
        results: list of dicts with keys {pose, q, fk_pos, fk_err_mm, label}
    """
    results = []
    clamped_count = 0
    unreachable_count = 0

    # Group poses by label to verify per-segment Z constancy
    segment_z = {}  # label → first z seen

    print("=" * 72)
    print("  Task 2.c — Simulation Run")
    print("=" * 72)

    for i, pose in enumerate(poses):
        x, y, z, p = pose
        label = labels[i] if labels else ""

        # ── Z-constancy assertion for XY segments ──
        if label.startswith("xy_") or label.startswith("gate_segment"):
            if label not in segment_z:
                segment_z[label] = z
            else:
                assert abs(z - segment_z[label]) < 1e-6, (
                    f"Z drift in {label}: expected {segment_z[label]}, got {z}"
                )

        # ── IK ──
        try:
            q = IK(x, y, z, p)
        except Exception as e:
            print(f"  [{i:4d}] UNREACHABLE  {label:24s}  pose=({x:.1f}, {y:.1f}, {z:.1f})  err={e}")
            unreachable_count += 1
            results.append(dict(pose=pose, q=None, fk_pos=None, fk_err_mm=None, label=label))
            continue

        # ── Joint limit check ──
        is_valid, violations = validate_joints(q)
        if not is_valid:
            clamped_count += 1

        # ── FK verification ──
        T_ee, _ = FK(q)
        fk_pos = T_ee[:3, 3]
        fk_err = np.linalg.norm(fk_pos - np.array([x, y, z]))

        results.append(dict(
            pose=pose, q=list(q), fk_pos=fk_pos.tolist(),
            fk_err_mm=float(fk_err), label=label,
        ))

    # ── Summary ──
    print()
    print("-" * 72)
    print(f"  Total poses:      {len(poses)}")
    print(f"  Unreachable:      {unreachable_count}")
    print(f"  Clamped joints:   {clamped_count}")

    fk_errs = [r["fk_err_mm"] for r in results if r["fk_err_mm"] is not None]
    if fk_errs:
        print(f"  FK error — mean:  {np.mean(fk_errs):.3f} mm")
        print(f"  FK error — max:   {np.max(fk_errs):.3f} mm")

    # ── Per-segment Z drift report ──
    print()
    print("  Per-segment Z-drift (commanded):")
    current_label = None
    seg_z_vals = []
    for r in results:
        lbl = r["label"]
        if lbl != current_label:
            if current_label and seg_z_vals:
                drift = max(seg_z_vals) - min(seg_z_vals)
                tag = "✓" if drift < 1e-6 else "✗"
                print(f"    {tag} {current_label:28s}  z_range=[{min(seg_z_vals):.3f}, {max(seg_z_vals):.3f}]  drift={drift:.6f}")
            current_label = lbl
            seg_z_vals = []
        seg_z_vals.append(r["pose"][2])

    # Flush last segment
    if current_label and seg_z_vals:
        drift = max(seg_z_vals) - min(seg_z_vals)
        tag = "✓" if drift < 1e-6 else "✗"
        print(f"    {tag} {current_label:28s}  z_range=[{min(seg_z_vals):.3f}, {max(seg_z_vals):.3f}]  drift={drift:.6f}")

    # ── FK Z drift for XY segments ──
    print()
    print("  FK-measured Z during XY segments (IK roundtrip):")
    current_label = None
    seg_fk_z = []
    for r in results:
        lbl = r["label"]
        if lbl.startswith("xy_") or lbl.startswith("gate_segment"):
            if lbl != current_label:
                if current_label and seg_fk_z:
                    drift = max(seg_fk_z) - min(seg_fk_z)
                    print(f"    {current_label:28s}  fk_z=[{min(seg_fk_z):.2f}, {max(seg_fk_z):.2f}]  drift={drift:.3f} mm")
                current_label = lbl
                seg_fk_z = []
            if r["fk_pos"] is not None:
                seg_fk_z.append(r["fk_pos"][2])

    if current_label and seg_fk_z:
        drift = max(seg_fk_z) - min(seg_fk_z)
        print(f"    {current_label:28s}  fk_z=[{min(seg_fk_z):.2f}, {max(seg_fk_z):.2f}]  drift={drift:.3f} mm")

    print()
    print("=" * 72)
    print("  Simulation complete.")
    print("=" * 72)

    return results


# ── Entry Point ───────────────────────────────────────────────────────

if __name__ == "__main__":
    poses, labels = build_task2c_path()
    results = simulate_path(poses, labels)
