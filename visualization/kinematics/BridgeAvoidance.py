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
import numpy as np


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
                          zone_x_min: float = 220.0,
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

    def _route_is_safe(start_wp, route_wps):
        prev = _as_pose4(start_wp)
        for wp in route_wps:
            curr = _as_pose4(wp)
            if _check_path_segment(prev, curr, collision_zones, samples):
                return False
            prev = curr
        return True
    
    # Use multi-zone if available, else single bounding box
    collision_zones = zones if zones else [zone]

    start_under = start[2] < zone.z_max
    target_under = target[2] < zone.z_max
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
    # Do not require near-horizontal start pitch here.
    # The solver can reach under-bridge states with non-zero pitch, and retract
    # should still use the safe staged exit route from any feasible start pose.
    if start_under and not target_under:
        empty_zones = [BridgeNoGoZone(0, 0, 0, 0, 0, 0)]
        exit_x = min(approach_x, zone.x_min - 27.0)  # prefer farther in-front exit
        min_reachable_z = _find_min_safe_z(
            exit_x, start[1], approach_pitch, empty_zones,
            z_low=10.0, z_high=safe_z, tol=2.0
        )
        drop_z = max(start[2], min_reachable_z + 5.0)

        # Candidate A: under-bridge slide out, then lift, then go target.
        wp_slide_out = [exit_x, start[1], drop_z, start[3]]
        wp_lift = [exit_x, start[1], safe_z, target[3]]
        route_a = _dedupe_waypoints([wp_slide_out, wp_lift, target])
        if _route_is_safe(start, route_a):
            return route_a

        # Candidate B: after lift, route via "over target XY" at safe Z.
        wp_over = [target[0], target[1], safe_z, target[3]]
        route_b = _dedupe_waypoints([wp_slide_out, wp_lift, wp_over, target])
        if _route_is_safe(start, route_b):
            return route_b

        # Candidate C: pure safe-Z dogleg fallback from current XY.
        wp_up = [start[0], start[1], safe_z, start[3]]
        route_c = _dedupe_waypoints([wp_up, wp_over, target])
        if _route_is_safe(start, route_c):
            return route_c

        # Last fallback: direct target if no safe staged route was found.
        return [target]

    # ---- UNDER->UNDER CASE: both poses below bridge deck ----
    # Avoid vertical moves while still inside bridge X-span by sliding out first.
    if start_under and target_under:
        exit_x = min(zone.x_min - 27.0, start[0], target[0])

        # Candidate A: slide out at start Z, then adjust to target Z outside bridge span.
        wp_slide_out = [exit_x, start[1], start[2], start[3]]
        wp_adjust = [exit_x, target[1], target[2], target[3]]
        route_a = _dedupe_waypoints([wp_slide_out, wp_adjust, target])
        if _route_is_safe(start, route_a):
            return route_a

        # Candidate B: first align XY at low Z, then rise at target XY if safe.
        wp_xy = [target[0], target[1], start[2], target[3]]
        route_b = _dedupe_waypoints([wp_xy, target])
        if _route_is_safe(start, route_b):
            return route_b

        # Last resort for this special case: keep staged under-bridge route shape.
        # This avoids collapsing into a straight direct move that often rises inside
        # bridge span and is more collision-prone in practice.
        return route_a

    # ---- ENTRY CASE: target is under bridge ----
    if target_under and target_horiz:
        close_approach_x = zone.x_min - 27.0  # just in front of bridge
        
        # Compute safe_z for this close approach X
        close_min_safe_z = _find_min_safe_z(
            close_approach_x, target[1], approach_pitch, collision_zones,
            z_low=10.0, z_high=zone.z_max + 80.0
        )
        close_safe_z = max(close_min_safe_z, zone.z_max) + abs(float(vertical_clearance_mm))
        
        mid_z = zone.z_max - 30.0
        mid_x = close_approach_x + 10.0
        diag_z = 50.0
        
        # Path: lift → sweep to approach → drop to MID_Z → diagonal down+forward → slide to target
        wp_lift = [start[0], start[1], close_safe_z, approach_pitch]
        wp_over = [close_approach_x, target[1], close_safe_z, approach_pitch]
        wp_mid  = [close_approach_x, target[1], mid_z, approach_pitch]
        wp_diag = [mid_x, target[1], diag_z, target[3]]
        
        return _dedupe_waypoints([wp_lift, wp_over, wp_mid, wp_diag, target])

    # Quick check: for non-special cases, allow direct path if safe.
    direct_hit = _check_path_segment(start, target, collision_zones, samples)
    if not direct_hit:
        return [target]

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


def solve_optimal_pitch(x, y, z, zones, preferred_pitch=0.0,
                        prev_pitch=None, max_pitch_rate=4.0,
                        pitch_range=(-60.0, 10.0), margin_deg=5.0,
                        terminal_target_pitch=None,
                        terminal_target_weight=200.0,
                        enforce_terminal_target_if_feasible=False,
                        bridge_proximity_weight=35.0,
                        bridge_proximity_decay_mm=12.0,
                        bridge_proximity_samples=4,
                        bridge_x_proximity_weight=55.0,
                        bridge_x_proximity_decay_mm=10.0):
    """
    Find the optimal pitch at position (x,y,z) by minimizing a cost function.
    
    The cost function penalizes:
      1. Approaching joint limits (exponentially).
      2. Approaching bridge collision boundaries.
      3. Large deviations from the previous pitch (smoothness).
      
    Returns optimal pitch in degrees, or None if unreachable.
    """
    import math
    zone_list = zones if isinstance(zones, list) else [zones]
    
    def _pitch_is_feasible(p):
        try:
            q = IK(x, y, z, p)
        except Exception:
            return False

        vmin = JOINT_LIMITS['min'] + margin_deg
        vmax = JOINT_LIMITS['max'] - margin_deg
        for i in range(len(q)):
            if q[i] <= vmin[i] or q[i] >= vmax[i]:
                return False

        try:
            _, transforms = FK(q)
            for j in range(len(transforms) - 1):
                p_start = transforms[j][:3, 3]
                p_end = transforms[j + 1][:3, 3]
                if segment_intersects_no_go_zone(
                    p_start, p_end, zone_list, samples=5, padding_mm=2.0
                ):
                    return False
        except Exception:
            return False

        return True

    def _point_to_zone_outside_deltas_mm(point_xyz, zone):
        """
        Axis-wise outside deltas from point to AABB.
        Returns (dx, dy, dz), where each term is 0 if inside interval on that axis.
        """
        px, py, pz = [float(v) for v in point_xyz]
        dx = 0.0
        if px < zone.x_min:
            dx = zone.x_min - px
        elif px > zone.x_max:
            dx = px - zone.x_max

        dy = 0.0
        if py < zone.y_min:
            dy = zone.y_min - py
        elif py > zone.y_max:
            dy = py - zone.y_max

        dz = 0.0
        if pz < zone.z_min:
            dz = zone.z_min - pz
        elif pz > zone.z_max:
            dz = pz - zone.z_max

        return float(dx), float(dy), float(dz)

    def _bridge_proximity_cost(transforms):
        """
        Soft penalty for getting close to bridge solids.
        Cost decays quickly with distance, so far-away links are almost free.
        """
        decay = max(1e-6, float(bridge_proximity_decay_mm))
        x_decay = max(1e-6, float(bridge_x_proximity_decay_mm))
        samples = max(2, int(bridge_proximity_samples))
        total_dist = 0.0
        total_x = 0.0
        count = 0

        for j in range(len(transforms) - 1):
            p_start = transforms[j][:3, 3]
            p_end = transforms[j + 1][:3, 3]
            for i in range(samples + 1):
                t = i / samples
                p = p_start + (p_end - p_start) * t
                min_dist = float("inf")
                min_dx = float("inf")
                for zone in zone_list:
                    dx, dy, dz = _point_to_zone_outside_deltas_mm(p, zone)
                    d = float(np.sqrt(dx * dx + dy * dy + dz * dz))
                    if d < min_dist:
                        min_dist = d
                    if dx < min_dx:
                        min_dx = dx

                # General closeness penalty (all axes).
                total_dist += np.exp(-min_dist / decay)
                # Extra X-direction proximity penalty to discourage approaching bridge span in X.
                total_x += np.exp(-min_dx / x_decay)
                count += 1

        if count == 0:
            return 0.0
        mean_dist = float(total_dist / count)
        mean_x = float(total_x / count)
        return (
            float(bridge_proximity_weight) * mean_dist
            + float(bridge_x_proximity_weight) * mean_x
        )

    def evaluate_cost(p):
        try:
            q = IK(x, y, z, p)
        except Exception:
            return float('inf')
            
        # 1. Joint Limit Cost
        # We want to exponentially penalize being close to the limits
        cost_limits = 0.0
        vmin = JOINT_LIMITS['min'] + margin_deg
        vmax = JOINT_LIMITS['max'] - margin_deg
        
        for i in range(len(q)):
            if q[i] <= vmin[i] or q[i] >= vmax[i]:
                return float('inf') # Hard violation of margin
            
            # Distance from center of joint range (normalized 0 to 1)
            center = (vmax[i] + vmin[i]) / 2.0
            range_half = (vmax[i] - vmin[i]) / 2.0
            norm_dist = abs(q[i] - center) / range_half
            
            # Exponential penalty as we approach the edge (norm_dist -> 1)
            cost_limits += math.exp(norm_dist * 5) - 1.0

        # 2. Collision Cost
        cost_collision = 0.0
        cost_proximity = 0.0
        try:
            T_ee, transforms = FK(q)
            for j in range(len(transforms) - 1):
                p_start = transforms[j][:3, 3]
                p_end = transforms[j+1][:3, 3]
                
                # Check for hard collision using a tighter margin for true safety
                if segment_intersects_no_go_zone(p_start, p_end, zone_list, samples=5, padding_mm=2.0):
                    return float('inf')

            # Soft "don't get too close" bridge penalty.
            cost_proximity = _bridge_proximity_cost(transforms)
        except Exception:
            return float('inf')

        # 3. Smoothness + Preference Cost
        cost_smoothness = 0.0
        if prev_pitch is not None:
            delta = abs(p - prev_pitch)
            if delta > max_pitch_rate:
                # Heavy penalty for exceeding max rate, but not infinite,
                # in case it's the ONLY way to avoid a collision.
                cost_smoothness += 1000.0 * (delta - max_pitch_rate)
            cost_smoothness += delta * 2.0  # Pull toward previous
        cost_smoothness += abs(p - preferred_pitch) * 0.5  # Gentle pull to preferred

        # 4. Terminal target preference (hardcoded high weight path support)
        cost_terminal = 0.0
        if terminal_target_pitch is not None:
            cost_terminal = abs(p - float(terminal_target_pitch)) * float(terminal_target_weight)

        return cost_limits + cost_collision + cost_proximity + cost_smoothness + cost_terminal

    # Generate candidates at 1-degree resolution
    p_min = int(math.floor(pitch_range[0]))
    p_max = int(math.ceil(pitch_range[1]))
    
    # Optional strict terminal lock: choose terminal target immediately if feasible.
    # This supports "final pick should be 0 deg whenever possible".
    if terminal_target_pitch is not None and enforce_terminal_target_if_feasible:
        p0 = float(terminal_target_pitch)
        if p_min <= p0 <= p_max and _pitch_is_feasible(p0):
            return p0

    best_pitch = None
    min_cost = float('inf')
    
    for p in range(p_min, p_max + 1):
        cost = evaluate_cost(float(p))
        if cost < min_cost:
            min_cost = cost
            best_pitch = float(p)
            
    # Try the exact previous/preferred pitches as candidates too
    for p in [prev_pitch, preferred_pitch]:
        if p is not None:
            cost = evaluate_cost(float(p))
            if cost < min_cost:
                min_cost = cost
                best_pitch = float(p)

    return best_pitch
