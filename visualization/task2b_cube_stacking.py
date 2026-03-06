"""
Task 2b – Pick-and-Rotate Simulation
=====================================
Generates a waypoint sequence that:
  1. Approaches above cube with pitch = 0
  2. Tilts pitch to -90 (vertical grip) while hovering
  3. Descends to pick height (z = cube centre, z_surface + size/2)
  4. Closes gripper
  5. Lifts the cube
  6. Rotates pitch in-place:  -90 → 0  (vertical to horizontal)
  7. Lowers cube back to original (x, y) spot  at pitch = 0
  8. Opens gripper
  9. Retracts and returns home

Each waypoint: WaypointAction(x, y, z, pitch, gripper_pct, label)
"""

from dataclasses import dataclass
from typing import List
import numpy as np


# ── Constants ──────────────────────────────────────────────────────
CUBE_SIZE_MM   = 25.0
CUBE_Z_SURFACE = 45.0        # Z of table surface (cube bottom face)
HOVER_HEIGHT   = 80.0        # clearance added above cube surface for hover
PICK_PITCH     = -90.0       # pitch for vertical (downward) grasp
GRIP_CLOSED    = 100         # gripper % fully closed
GRIP_OPEN      = 0
ROTATE_STEPS   = 30          # frames for pitch sweep
MOVE_STEPS     = 10          # frames for linear moves


@dataclass
class WaypointAction:
    x: float
    y: float
    z: float
    pitch: float
    gripper_pct: int
    label: str


def _lerp(start: WaypointAction, end: WaypointAction,
          steps: int, label: str) -> List[WaypointAction]:
    """Linearly interpolate between two waypoints."""
    wps = []
    for i in range(1, steps + 1):
        t = i / steps
        wps.append(WaypointAction(
            x           = start.x     + (end.x     - start.x)     * t,
            y           = start.y     + (end.y     - start.y)     * t,
            z           = start.z     + (end.z     - start.z)     * t,
            pitch       = start.pitch + (end.pitch - start.pitch) * t,
            gripper_pct = end.gripper_pct,
            label       = label,
        ))
    return wps


def generate_pick_rotate_sequence(
    pick_x: float = 200.0,
    pick_y: float = 0.0,
    place_x: float | None = None,
    place_y: float | None = None,
    cube_z_surface: float = CUBE_Z_SURFACE,
    cube_size: float = CUBE_SIZE_MM,
    rotate_mode: int | bool = 1,
) -> List[WaypointAction]:
    """
    Pick a cube at (pick_x, pick_y, cube_z_surface) with a vertical grip
    (pitch = -90), optionally rotate the pitch by +90° in place, and place
    it down at (place_x, place_y).

    If the pick point is too close to the base for a safe rotation
    (< MIN_ROTATE_RADIUS mm), the arm moves outward first to a safe
    radial distance before rotating.
    """
    # Default place position to pick position if not specified
    if place_x is None:
        place_x = pick_x
    if place_y is None:
        place_y = pick_y

    # Normalise rotate mode:
    #   0 -> no rotation,
    #   +1 -> +90°,
    #   -1 -> -90°.
    if isinstance(rotate_mode, bool):
        rotate_mode = 1 if rotate_mode else 0
    rotate_mode = int(np.sign(rotate_mode)) if rotate_mode != 0 else 0

    # Choose pick pitch based on rotation mode:
    #   rotate_mode >= 0 → pick at -90°, rotate -90 → 0 (if enabled)
    #   rotate_mode < 0  → pick at 0°,   rotate 0   → -90° (if enabled)
    if rotate_mode < 0:
        pick_pitch = 0.0
    else:
        pick_pitch = PICK_PITCH

    pick_z = cube_z_surface + cube_size / 2   # grasp at cube centre
    place_z = pick_z
    hover_z = cube_z_surface + HOVER_HEIGHT

    # Minimum radial distance to safely rotate.
    # Tool length is 126mm; wrist centre needs ≥ ~60mm radially.
    MIN_ROTATE_RADIUS = 200.0

    r_pick = np.sqrt(pick_x**2 + pick_y**2)

    # If the cube is too close, compute a safe point on the same radial line
    if r_pick < MIN_ROTATE_RADIUS and r_pick > 1e-6:
        scale = MIN_ROTATE_RADIUS / r_pick
        safe_x = pick_x * scale
        safe_y = pick_y * scale
    else:
        safe_x = pick_x
        safe_y = pick_y

    needs_pullout = (r_pick < MIN_ROTATE_RADIUS)

    wps: List[WaypointAction] = []

    # Home
    home = WaypointAction(200, 0, 150, 0, GRIP_OPEN, "Home")
    wps.append(home)

    if needs_pullout:
        # Approach via safe radius: stay at safe_x/y with pitch=0, then
        # tilt to -90, THEN travel inward to pick (all at pitch=-90).
        above_safe = WaypointAction(safe_x, safe_y, hover_z, 0, GRIP_OPEN,
                                    "Approaching safe radius")
        wps += _lerp(home, above_safe, MOVE_STEPS, "Moving to safe radius")

        above_safe_v = WaypointAction(safe_x, safe_y, hover_z, pick_pitch,
                                      GRIP_OPEN, "Tilting to vertical grip")
        wps += _lerp(above_safe, above_safe_v, MOVE_STEPS,
                     "Tilting to vertical grip")

        above_pick_v = WaypointAction(pick_x, pick_y, hover_z, pick_pitch,
                                      GRIP_OPEN, "Above pick (vertical)")
        wps += _lerp(above_safe_v, above_pick_v, MOVE_STEPS,
                     "Moving above pick")
    else:
        # Safe to approach at pitch=0 and tilt in place
        above = WaypointAction(pick_x, pick_y, hover_z, 0, GRIP_OPEN,
                               "Approaching above pick")
        wps += _lerp(home, above, MOVE_STEPS, "Moving above pick")

        above_pick_v = WaypointAction(pick_x, pick_y, hover_z, pick_pitch,
                                      GRIP_OPEN, "Tilting to vertical grip")
        wps += _lerp(above, above_pick_v, MOVE_STEPS,
                     "Tilting to vertical grip")

    # 3. Descend to pick
    at_pick = WaypointAction(pick_x, pick_y, pick_z, pick_pitch,
                             GRIP_OPEN, "At pick – open gripper")
    wps += _lerp(above_pick_v, at_pick, MOVE_STEPS, "Descending to pick")

    # 4. Close gripper
    gripped = WaypointAction(pick_x, pick_y, pick_z, pick_pitch,
                             GRIP_CLOSED, "Gripping cube")
    for _ in range(3):
        wps.append(gripped)

    # 5. Lift (pitch unchanged)
    lifted = WaypointAction(pick_x, pick_y, hover_z, pick_pitch,
                            GRIP_CLOSED, "Lifting cube")
    wps += _lerp(gripped, lifted, MOVE_STEPS, "Lifting cube")

    # 5b. If too close, pull out to safe radius before rotating
    if needs_pullout:
        safe_pos = WaypointAction(safe_x, safe_y, hover_z, pick_pitch,
                                  GRIP_CLOSED, "Moving to safe radius")
        wps += _lerp(lifted, safe_pos, MOVE_STEPS, "Moving to safe radius")
        rotate_from = safe_pos
    else:
        rotate_from = lifted

    # 6. Optionally rotate pitch in place (at safe radius or lifted pos)
    if rotate_mode != 0:
        if rotate_mode > 0:
            # -90 → 0
            final_pitch = 0.0
        else:
            # 0 → -90
            final_pitch = -90.0
        rotated = WaypointAction(
            rotate_from.x,
            rotate_from.y,
            hover_z,
            final_pitch,
            GRIP_CLOSED,
            f"Rotated – pitch now {final_pitch:.1f}°",
        )
        wps += _lerp(
            rotate_from,
            rotated,
            ROTATE_STEPS,
            "Rotating cube",
        )
    else:
        rotated = rotate_from
        final_pitch = rotate_from.pitch

    # 7. Move above place position at final pitch
    above_place = WaypointAction(
        place_x,
        place_y,
        hover_z,
        final_pitch,
        GRIP_CLOSED,
        "Moving above place",
    )
    wps += _lerp(rotated, above_place, MOVE_STEPS, "Moving above place")

    # 8. Descend to place
    at_place = WaypointAction(
        place_x,
        place_y,
        place_z,
        final_pitch,
        GRIP_CLOSED,
        "Lowering cube to place",
    )
    wps += _lerp(above_place, at_place, MOVE_STEPS, "Lowering cube")

    # 9. Open gripper
    released = WaypointAction(
        place_x,
        place_y,
        place_z,
        final_pitch,
        GRIP_OPEN,
        "Releasing cube",
    )
    for _ in range(3):
        wps.append(released)

    # 10. Retract upward
    above_place_up = WaypointAction(
        place_x,
        place_y,
        hover_z,
        final_pitch,
        GRIP_OPEN,
        "Retracting",
    )
    wps += _lerp(released, above_place_up, MOVE_STEPS, "Retracting")

    # 11. Return home
    wps += _lerp(above_place_up, home, MOVE_STEPS, "Returning home")

    return wps


# ── Quick smoke test ───────────────────────────────────────────────
if __name__ == "__main__":
    seq = generate_pick_rotate_sequence(200, 0)
    print(f"Generated {len(seq)} waypoints:")
    for i, wp in enumerate(seq):
        print(f"  [{i:3d}] ({wp.x:7.1f}, {wp.y:7.1f}, {wp.z:5.1f})"
              f"  pitch={wp.pitch:6.1f}  grip={wp.gripper_pct:3d}%  {wp.label}")
