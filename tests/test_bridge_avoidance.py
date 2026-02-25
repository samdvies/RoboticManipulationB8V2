"""
Tests for bridge no-go routing and bridge gripper width constraints.
"""
import os
import sys

# Ensure imports work from repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from visualization.kinematics.BridgeAvoidance import (
    BridgeNoGoZone,
    clamp_gripper_width_for_bridge,
    plan_bridge_safe_waypoints,
    segment_intersects_no_go_zone,
)


def _is_close(a, b, tol=1e-6):
    return abs(float(a) - float(b)) <= tol


def _pose_close(a, b, tol=1e-6):
    return all(_is_close(x, y, tol) for x, y in zip(a, b))


def test_segment_intersection_detects_bridge_hit():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    start = [200.0, 0.0, 180.0]
    end = [200.0, 0.0, 50.0]
    assert segment_intersects_no_go_zone(start, end, zone)


def test_segment_intersection_below_bridge_is_safe():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    start = [260.0, 0.0, 50.0]
    end = [200.0, 0.0, 50.0]
    assert not segment_intersects_no_go_zone(start, end, zone)


def test_plan_route_adds_detour_for_under_bridge_pick_at_zero_pitch():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    start_pose = [200.0, 0.0, 180.0, 0.0]
    target_pose = [200.0, 0.0, 50.0, 0.0]

    waypoints = plan_bridge_safe_waypoints(start_pose, target_pose, zone)

    # Detour expected: pre-bridge high, pre-bridge low, final target
    assert len(waypoints) == 3
    assert _pose_close(waypoints[-1], target_pose)

    # First waypoint should be outside x-span of the bridge and above it
    wp0 = waypoints[0]
    assert wp0[0] > zone.x_max or wp0[0] < zone.x_min
    assert wp0[2] > zone.z_max

    # Second waypoint should keep same x/y/pitch and drop to target Z
    wp1 = waypoints[1]
    assert _is_close(wp1[0], wp0[0])
    assert _is_close(wp1[1], target_pose[1])
    assert _is_close(wp1[2], target_pose[2])
    assert _is_close(wp1[3], target_pose[3])


def test_plan_route_keeps_direct_path_when_not_bridge_pick():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    start_pose = [200.0, 0.0, 180.0, 0.0]
    target_pose = [200.0, 0.0, 140.0, -30.0]  # not under bridge and not pitch 0

    waypoints = plan_bridge_safe_waypoints(start_pose, target_pose, zone)
    assert len(waypoints) == 1
    assert _pose_close(waypoints[0], target_pose)


def test_gripper_width_clamped_to_bridge_band():
    assert _is_close(clamp_gripper_width_for_bridge(10.0), 25.0)
    assert _is_close(clamp_gripper_width_for_bridge(32.0), 32.0)
    assert _is_close(clamp_gripper_width_for_bridge(60.0), 50.0)


if __name__ == "__main__":
    tests = [
        test_segment_intersection_detects_bridge_hit,
        test_segment_intersection_below_bridge_is_safe,
        test_plan_route_adds_detour_for_under_bridge_pick_at_zero_pitch,
        test_plan_route_keeps_direct_path_when_not_bridge_pick,
        test_gripper_width_clamped_to_bridge_band,
    ]
    passed = 0
    failed = 0
    for test_fn in tests:
        try:
            test_fn()
            print(f"PASS: {test_fn.__name__}")
            passed += 1
        except Exception as exc:
            print(f"FAIL: {test_fn.__name__}: {exc}")
            failed += 1
    print(f"\nResults: {passed} passed, {failed} failed out of {len(tests)}")
    if failed:
        raise SystemExit(1)
