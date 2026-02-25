"""
Bridge avoidance helpers for Python simulation motion planning.

Provides:
1) A configurable no-go box representing bridge volume.
2) Multi-zone support: bridge is decomposed into solid parts (pillars + deck).
3) Segment/zone intersection sampling checks.
4) Under-bridge waypoint planning for horizontal (pitch~0) pickups.
5) Gripper jaw-width clamping for bridge clearance.
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class BridgeNoGoZone:
    """Axis-aligned no-go zone in task space (millimeters)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    def center_x(self) -> float:
        return 0.5 * (self.x_min + self.x_max)

    def contains_point(self, point_xyz, padding_mm: float = 0.0) -> bool:
        x, y, z = [float(v) for v in point_xyz]
        p = max(0.0, float(padding_mm))
        return (
            (self.x_min - p) <= x <= (self.x_max + p)
            and (self.y_min - p) <= y <= (self.y_max + p)
            and (self.z_min - p) <= z <= (self.z_max + p)
        )


def build_bridge_zones(zone: BridgeNoGoZone,
                       gap_y: float = 50.0,
                       pillar_width_y: float = 10.0,
                       deck_thickness: float = 10.0) -> List[BridgeNoGoZone]:
    """
    Decompose a bridge bounding box into its solid components.
    
    Returns a list of 3 BridgeNoGoZone objects:
      - Left pillar  (Y < -gap/2)
      - Right pillar  (Y > +gap/2)
      - Top deck beam (full Y span, only top slice of Z)
    
    The gap between pillars (Y = -gap/2 to +gap/2 at Z < deck) is passable.
    """
    half_gap = gap_y / 2.0
    pw = pillar_width_y
    
    left_pillar = BridgeNoGoZone(
        x_min=zone.x_min, x_max=zone.x_max,
        y_min=-half_gap - pw, y_max=-half_gap,
        z_min=zone.z_min, z_max=zone.z_max
    )
    right_pillar = BridgeNoGoZone(
        x_min=zone.x_min, x_max=zone.x_max,
        y_min=half_gap, y_max=half_gap + pw,
        z_min=zone.z_min, z_max=zone.z_max
    )
    deck = BridgeNoGoZone(
        x_min=zone.x_min, x_max=zone.x_max,
        y_min=-half_gap - pw, y_max=half_gap + pw,
        z_min=zone.z_max - deck_thickness, z_max=zone.z_max
    )
    return [left_pillar, right_pillar, deck]


def _as_pose4(pose) -> list:
    vals = [float(v) for v in pose]
    if len(vals) != 4:
        raise ValueError(f"Expected pose with 4 values [x,y,z,pitch], got {len(vals)}.")
    return vals


def _as_xyz(p) -> list:
    vals = [float(v) for v in p]
    if len(vals) < 3:
        raise ValueError(f"Expected at least 3 values [x,y,z], got {len(vals)}.")
    return vals[:3]


def _lerp_xyz(start_xyz, end_xyz, t: float) -> list:
    return [
        start_xyz[0] + (end_xyz[0] - start_xyz[0]) * t,
        start_xyz[1] + (end_xyz[1] - start_xyz[1]) * t,
        start_xyz[2] + (end_xyz[2] - start_xyz[2]) * t,
    ]


def segment_intersects_no_go_zone(
    start_xyz,
    end_xyz,
    zone,  # BridgeNoGoZone or list of BridgeNoGoZone
    samples: int = 64,
    padding_mm: float = 0.0,
) -> bool:
    """Sample along segment and detect entry into any no-go zone."""
    s = _as_xyz(start_xyz)
    e = _as_xyz(end_xyz)
    n = max(2, int(samples))
    
    # Support both single zone and list of zones
    zones = zone if isinstance(zone, list) else [zone]
    
    for i in range(n + 1):
        t = i / n
        p = _lerp_xyz(s, e, t)
        for z in zones:
            if z.contains_point(p, padding_mm=padding_mm):
                return True
    return False


def _dedupe_waypoints(waypoints, tol: float = 1e-6) -> list:
    out = []
    for wp in waypoints:
        wp4 = _as_pose4(wp)
        if not out:
            out.append(wp4)
            continue
        prev = out[-1]
        if all(abs(prev[i] - wp4[i]) <= tol for i in range(4)):
            continue
        out.append(wp4)
    return out


from .IK import IK
from .FK import FK
from .JointLimits import validate_joints, JOINT_LIMITS

def _check_path_segment(start_pose, end_pose, zones, samples: int = 20) -> bool:
    """Returns True if any part of the arm intersects any zone or if the path is unreachable.
    
    zones: single BridgeNoGoZone or list of BridgeNoGoZone
    """
    start = _as_pose4(start_pose)
    end = _as_pose4(end_pose)
    zone_list = zones if isinstance(zones, list) else [zones]
    
    for i in range(samples + 1):
        t = i / max(1, samples)
        p = _lerp_xyz(start[:3], end[:3], t)
        pitch = start[3] + (end[3] - start[3]) * t
        
        try:
            q = IK(p[0], p[1], p[2], pitch)
            is_valid, _ = validate_joints(q, tolerance_deg=0)
            if not is_valid:
                return True
                    
            T_ee, transforms = FK(q)
            for j in range(len(transforms) - 1):
                p_start = transforms[j][:3, 3]
                p_end = transforms[j+1][:3, 3]
                if segment_intersects_no_go_zone(p_start, p_end, zone_list, samples=10):
                    return True
        except Exception:
            return True
    return False


def _arm_safe_at(x, y, z, pitch, zones) -> bool:
    """Check if the arm at this single pose is collision-free and within joint limits."""
    zone_list = zones if isinstance(zones, list) else [zones]
    try:
        q = IK(x, y, z, pitch)
        is_valid, _ = validate_joints(q, tolerance_deg=0)
        if not is_valid:
            return False
        T_ee, transforms = FK(q)
        for j in range(len(transforms) - 1):
            p_start = transforms[j][:3, 3]
            p_end = transforms[j+1][:3, 3]
            if segment_intersects_no_go_zone(p_start, p_end, zone_list, samples=10):
                return False
        return True
    except Exception:
        return False


def _find_min_safe_z(x, y, pitch, zones,
                     z_low: float = 10.0, z_high: float = 300.0,
                     tol: float = 2.0) -> float:
    """
    Binary-search for the minimum Z at which the arm (at position x, y, pitch)
    has all links clear of all zones AND stays within joint limits.
    """
    if not _arm_safe_at(x, y, z_high, pitch, zones):
        return z_high
    
    lo, hi = z_low, z_high
    for _ in range(20):
        if (hi - lo) < tol:
            break
        mid = (lo + hi) / 2.0
        if _arm_safe_at(x, y, mid, pitch, zones):
            hi = mid
        else:
            lo = mid
    return hi


def _find_safe_approach_x(target_z, target_y, pitch, zones,
                          x_min: float = 50.0, x_max_offset: float = 10.0,
                          zone_x_min: float = 187.5,
                          tol: float = 2.0) -> float:
    """
    Find the closest approach X (in front of the bridge) where the arm can
    execute the full vertical drop from above the bridge to target_z.
    """
    x_right = zone_x_min - x_max_offset
    x_left = x_min
    
    drop_z_top = 90.0  # drop from well above the bridge
    
    lo, hi = x_left, x_right
    best_x = x_left
    
    for _ in range(20):
        if (hi - lo) < tol:
            break
        mid = (lo + hi) / 2.0
        drop_start = [mid, target_y, drop_z_top, pitch]
        drop_end = [mid, target_y, target_z, pitch]
        drop_ok = not _check_path_segment(drop_start, drop_end, zones, samples=20)
        
        if drop_ok:
            best_x = mid
            lo = mid
        else:
            hi = mid
    
    return best_x


def plan_bridge_safe_waypoints(
    start_pose,
    target_pose,
    zone: BridgeNoGoZone,
    zones: Optional[List[BridgeNoGoZone]] = None,
    pitch_tolerance_deg: float = 5.0,
    approach_margin_mm: float = 60.0,
    vertical_clearance_mm: float = 30.0,
    samples: int = 20,
) -> list:
    """
    Plan a collision-free waypoint path from start to target, avoiding the bridge.
    
    If `zones` is provided (list of solid parts), uses multi-zone collision checking
    which allows the arm to pass through the gap between pillars.
    Otherwise falls back to treating the entire bounding box as solid.
    """
    start = _as_pose4(start_pose)
    target = _as_pose4(target_pose)
    
    # Use multi-zone if available, else single bounding box
    collision_zones = zones if zones else [zone]

    # Quick check: is the direct path already safe?
    direct_hit = _check_path_segment(start, target, collision_zones, samples)
    if not direct_hit:
        return [target]

    start_under = start[2] < zone.z_max
    target_under = target[2] < zone.z_max
    start_horiz = abs(start[3]) <= abs(float(pitch_tolerance_deg))
    target_horiz = abs(target[3]) <= abs(float(pitch_tolerance_deg))

    approach_pitch = 0.0

    # Find the optimal approach X
    approach_x = _find_safe_approach_x(
        target[2], target[1], approach_pitch, collision_zones,
        x_min=80.0, x_max_offset=10.0,
        zone_x_min=zone.x_min
    )
    
    # Compute minimum safe Z at the approach position
    min_safe_z = _find_min_safe_z(
        approach_x, target[1], approach_pitch, collision_zones,
        z_low=10.0, z_high=zone.z_max + 80.0
    )
    safe_z = max(min_safe_z, zone.z_max) + abs(float(vertical_clearance_mm))

    # ---- EXIT CASE: start is under bridge, target is above ----
    if start_under and not target_under and start_horiz:
        empty_zones = [BridgeNoGoZone(0, 0, 0, 0, 0, 0)]
        min_reachable_z = _find_min_safe_z(
            approach_x, start[1], approach_pitch, empty_zones,
            z_low=10.0, z_high=safe_z, tol=2.0
        )
        drop_z = max(start[2], min_reachable_z + 5.0)
        
        wp_slide_out = [approach_x, start[1], drop_z, start[3]]
        wp_lift = [approach_x, start[1], safe_z, target[3]]
        
        ok = (not _check_path_segment(start, wp_slide_out, collision_zones, samples) and
              not _check_path_segment(wp_slide_out, wp_lift, collision_zones, samples) and
              not _check_path_segment(wp_lift, target, collision_zones, samples))
        if ok:
            return _dedupe_waypoints([wp_slide_out, wp_lift, target])
        return _dedupe_waypoints([wp_slide_out, wp_lift, target])

    # ---- ENTRY CASE: target is under bridge ----
    if target_under and target_horiz:
        # Mirror the exit strategy: approach from just in front of the bridge,
        # drop to the lowest reachable Z there, then slide diagonally to target.
        # This keeps joints in comfortable ranges (no extreme shoulder angles).
        close_approach_x = zone.x_min - 12.0  # just in front of bridge
        
        # Find min reachable Z at close approach X (joint limits only, no bridge)
        empty_zones = [BridgeNoGoZone(0, 0, 0, 0, 0, 0)]
        min_reachable_z = _find_min_safe_z(
            close_approach_x, target[1], approach_pitch, empty_zones,
            z_low=10.0, z_high=safe_z, tol=2.0
        )
        drop_z = max(target[2], min_reachable_z + 5.0)
        
        # Compute safe_z for this close approach X
        close_min_safe_z = _find_min_safe_z(
            close_approach_x, target[1], approach_pitch, collision_zones,
            z_low=10.0, z_high=zone.z_max + 80.0
        )
        close_safe_z = max(close_min_safe_z, zone.z_max) + abs(float(vertical_clearance_mm))
        
        # Path: lift → move to approach X at safe Z → drop to reachable Z → slide to target
        wp_lift = [start[0], start[1], close_safe_z, approach_pitch]
        wp_over = [close_approach_x, target[1], close_safe_z, approach_pitch]
        wp_low = [close_approach_x, target[1], drop_z, approach_pitch]
        
        ok = (not _check_path_segment(start, wp_lift, collision_zones, samples) and
              not _check_path_segment(wp_lift, wp_over, collision_zones, samples) and
              not _check_path_segment(wp_over, wp_low, collision_zones, samples) and
              not _check_path_segment(wp_low, target, collision_zones, samples))
        if ok:
            return _dedupe_waypoints([wp_lift, wp_over, wp_low, target])
        return _dedupe_waypoints([wp_lift, wp_over, wp_low, target])

    # ---- GENERAL CASE ----
    wp_up = [start[0], start[1], safe_z, start[3]]
    wp_over = [target[0], target[1], safe_z, target[3]]
    ok = (not _check_path_segment(start, wp_up, collision_zones, samples) and
          not _check_path_segment(wp_up, wp_over, collision_zones, samples) and
          not _check_path_segment(wp_over, target, collision_zones, samples))
    if ok:
        return _dedupe_waypoints([wp_up, wp_over, target])
    
    return [target]


def clamp_gripper_width_for_bridge(
    jaw_width_mm: float,
    min_width_mm: float = 25.0,
    max_width_mm: float = 50.0,
) -> float:
    """Clamp jaw width into bridge pass-through band."""
    jaw = float(jaw_width_mm)
    lo = float(min_width_mm)
    hi = float(max_width_mm)
    if lo > hi:
        lo, hi = hi, lo
    if jaw < lo:
        return lo
    if jaw > hi:
        return hi
    return jaw
