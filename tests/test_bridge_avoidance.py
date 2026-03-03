"""
Tests for bridge no-go routing and bridge gripper width constraints.
"""
import os
import sys
import numpy as np
from unittest.mock import patch

# Ensure imports work from repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from visualization.kinematics.BridgeAvoidance import (
    BridgeNoGoZone,
    clamp_gripper_width_for_bridge,
    plan_bridge_safe_waypoints,
    segment_intersects_no_go_zone,
    solve_optimal_pitch,
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

    # Detour expected: lift, sweep over, drop, diagonal, target
    assert len(waypoints) == 5
    assert _pose_close(waypoints[-1], target_pose)

    wp_lift, wp_over, wp_mid, wp_diag, wp_tgt = waypoints

    # wp_lift: Keep X/Y, lift above bridge
    assert _is_close(wp_lift[0], start_pose[0])
    assert wp_lift[2] > zone.z_max

    # wp_over: approach X, safe Z
    assert wp_over[0] < zone.x_min
    assert wp_over[2] > zone.z_max

    # wp_mid: drop below deck at approach X
    assert _is_close(wp_mid[0], wp_over[0])
    assert wp_mid[2] < zone.z_max

    # wp_diag: intermediate diagonal step
    assert wp_diag[0] > wp_mid[0]
    assert wp_diag[2] < wp_mid[2]



def test_plan_route_keeps_direct_path_when_not_bridge_pick():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    start_pose = [200.0, 0.0, 180.0, 0.0]
    target_pose = [200.0, 0.0, 140.0, -30.0]  # not under bridge and not pitch 0

    waypoints = plan_bridge_safe_waypoints(start_pose, target_pose, zone)
    assert len(waypoints) == 1
    assert _pose_close(waypoints[0], target_pose)


def test_plan_route_exit_under_bridge_works_for_nonzero_start_pitch():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    # Start under bridge with non-zero pitch, retract to above bridge.
    start_pose = [200.0, 0.0, 50.0, -25.0]
    target_pose = [260.0, 0.0, 180.0, 0.0]

    waypoints = plan_bridge_safe_waypoints(start_pose, target_pose, zone)

    # Exit route should still be staged (not a direct single segment).
    assert len(waypoints) >= 2
    assert _pose_close(waypoints[-1], target_pose)


def test_plan_route_exit_does_not_return_known_unsafe_candidate():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    start_pose = [200.0, 0.0, 50.0, -25.0]
    target_pose = [260.0, 0.0, 180.0, 0.0]

    # First few calls correspond to Candidate A checks; force those unsafe,
    # then allow later calls so planner must choose a different route.
    call_counter = {"n": 0}

    def check_path_side_effect(*_args, **_kwargs):
        call_counter["n"] += 1
        return call_counter["n"] <= 3

    with patch("visualization.kinematics.BridgeAvoidance._check_path_segment", side_effect=check_path_side_effect):
        waypoints = plan_bridge_safe_waypoints(start_pose, target_pose, zone)
        assert len(waypoints) >= 2
        assert _pose_close(waypoints[-1], target_pose)


def test_plan_route_under_to_under_slides_out_before_rising():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    start_pose = [225.0, 0.0, 30.0, 0.0]
    target_pose = [200.0, 0.0, 100.0, 0.0]

    waypoints = plan_bridge_safe_waypoints(start_pose, target_pose, zone)
    assert len(waypoints) >= 2
    assert _pose_close(waypoints[-1], target_pose)
    # First routed waypoint should be outside bridge X-span (slide-out behavior).
    assert waypoints[0][0] < zone.x_min


def test_gripper_width_clamped_to_bridge_band():
    assert _is_close(clamp_gripper_width_for_bridge(10.0), 25.0)
    assert _is_close(clamp_gripper_width_for_bridge(32.0), 32.0)
    assert _is_close(clamp_gripper_width_for_bridge(60.0), 50.0)


def test_solve_optimal_pitch_terminal_prefers_zero_when_feasible():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    fake_tf = np.eye(4)
    fake_transforms = [fake_tf.copy(), fake_tf.copy()]
    with patch("visualization.kinematics.BridgeAvoidance.IK", return_value=[0.0, 0.0, 0.0, 0.0]), \
         patch("visualization.kinematics.BridgeAvoidance.FK", return_value=(fake_tf, fake_transforms)), \
         patch("visualization.kinematics.BridgeAvoidance.segment_intersects_no_go_zone", return_value=False):
        p = solve_optimal_pitch(
            225.0, 0.0, 30.0,
            zones=[zone],
            preferred_pitch=-20.0,
            prev_pitch=-20.0,
            terminal_target_pitch=0.0,
            terminal_target_weight=200.0,
            enforce_terminal_target_if_feasible=True,
        )
        assert _is_close(p, 0.0)


def test_solve_optimal_pitch_terminal_falls_back_when_zero_infeasible():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    fake_tf = np.eye(4)
    fake_transforms = [fake_tf.copy(), fake_tf.copy()]

    def ik_side_effect(x, y, z, pitch, *args, **kwargs):
        # Force 0 deg to violate joint limits while allowing negative pitches.
        if abs(float(pitch)) < 1e-6:
            return [200.0, 0.0, 0.0, 0.0]  # q1 beyond limit margin => infeasible
        return [0.0, 0.0, 0.0, 0.0]

    with patch("visualization.kinematics.BridgeAvoidance.IK", side_effect=ik_side_effect), \
         patch("visualization.kinematics.BridgeAvoidance.FK", return_value=(fake_tf, fake_transforms)), \
         patch("visualization.kinematics.BridgeAvoidance.segment_intersects_no_go_zone", return_value=False):
        p = solve_optimal_pitch(
            225.0, 0.0, 30.0,
            zones=[zone],
            preferred_pitch=-15.0,
            prev_pitch=-20.0,
            terminal_target_pitch=0.0,
            terminal_target_weight=200.0,
            enforce_terminal_target_if_feasible=True,
        )
        assert p is not None
        assert not _is_close(p, 0.0)


def test_solve_optimal_pitch_prefers_more_clearance_when_feasible():
    zone = BridgeNoGoZone(170, 230, -60, 60, 90, 130)
    fake_tf = np.eye(4)

    def ik_side_effect(x, y, z, pitch, *args, **kwargs):
        # Encode pitch into q[0] so FK can distinguish candidates.
        return [float(pitch), 0.0, 0.0, 0.0]

    def fk_side_effect(q):
        q0 = float(q[0])
        # For pitch near 0 deg, make link much closer to bridge deck.
        # For more negative pitch, keep link farther from bridge.
        z_link = 88.0 if abs(q0) < 1.0 else 40.0
        t0 = fake_tf.copy()
        t1 = fake_tf.copy()
        t0[:3, 3] = np.array([200.0, 0.0, z_link])
        t1[:3, 3] = np.array([210.0, 0.0, z_link])
        return fake_tf, [t0, t1]

    with patch("visualization.kinematics.BridgeAvoidance.IK", side_effect=ik_side_effect), \
         patch("visualization.kinematics.BridgeAvoidance.FK", side_effect=fk_side_effect), \
         patch("visualization.kinematics.BridgeAvoidance.segment_intersects_no_go_zone", return_value=False):
        p = solve_optimal_pitch(
            225.0, 0.0, 30.0,
            zones=[zone],
            preferred_pitch=0.0,
            prev_pitch=None,
            pitch_range=(-20.0, 0.0),
            bridge_proximity_weight=100.0,
            bridge_proximity_decay_mm=10.0,
        )
        assert p is not None
        # Should avoid the near-deck 0 deg candidate when a safer option exists.
        assert p <= -1.0


def test_solve_optimal_pitch_tracks_far_terminal_target_not_stuck_at_prev_pitch():
    zone = BridgeNoGoZone(220, 230, -35, 35, 90, 105)
    fake_tf = np.eye(4)
    fake_transforms = [fake_tf.copy(), fake_tf.copy()]

    with patch("visualization.kinematics.BridgeAvoidance.IK", return_value=[0.0, 0.0, 0.0, 0.0]), \
         patch("visualization.kinematics.BridgeAvoidance.FK", return_value=(fake_tf, fake_transforms)), \
         patch("visualization.kinematics.BridgeAvoidance.segment_intersects_no_go_zone", return_value=False):
        p = solve_optimal_pitch(
            150.0, 100.0, 30.0,
            zones=[zone],
            preferred_pitch=-90.0,
            prev_pitch=-60.0,
            max_pitch_rate=0.6,
            pitch_range=(-90.0, 10.0),
            terminal_target_pitch=-90.0,
            terminal_target_weight=240.0,
            enforce_terminal_target_if_feasible=False,
        )
        assert p is not None
        # Should move toward target without getting pinned at previous pitch.
        assert p < -60.0


if __name__ == "__main__":
    tests = [
        test_segment_intersection_detects_bridge_hit,
        test_segment_intersection_below_bridge_is_safe,
        test_plan_route_adds_detour_for_under_bridge_pick_at_zero_pitch,
        test_plan_route_keeps_direct_path_when_not_bridge_pick,
        test_plan_route_exit_under_bridge_works_for_nonzero_start_pitch,
        test_plan_route_exit_does_not_return_known_unsafe_candidate,
        test_plan_route_under_to_under_slides_out_before_rising,
        test_gripper_width_clamped_to_bridge_band,
        test_solve_optimal_pitch_terminal_prefers_zero_when_feasible,
        test_solve_optimal_pitch_terminal_falls_back_when_zero_infeasible,
        test_solve_optimal_pitch_prefers_more_clearance_when_feasible,
        test_solve_optimal_pitch_tracks_far_terminal_target_not_stuck_at_prev_pitch,
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
