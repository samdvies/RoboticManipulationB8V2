"""
Tests for the JointLimits module.
"""
import sys
import os
import numpy as np

# Ensure imports work
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from visualization.kinematics.JointLimits import clamp_joints, validate_joints, get_limits, JOINT_LIMITS
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK


def test_clamp_within_limits():
    """Angles within limits should pass through unchanged."""
    q = [0.0, 0.0, 0.0, 0.0]
    q_clamped, was_clamped = clamp_joints(q)
    assert q_clamped == q, f"Expected {q}, got {q_clamped}"
    assert not any(was_clamped), f"No joints should be clamped, got {was_clamped}"
    print("PASS: test_clamp_within_limits")


def test_clamp_above_max():
    """Angles above max should be clamped down."""
    q = [100.0, 130.0, 0.0, 0.0]
    q_clamped, was_clamped = clamp_joints(q)
    assert q_clamped[0] == 90.0, f"J1 should be clamped to 90, got {q_clamped[0]}"
    assert q_clamped[1] == 117.0, f"J2 should be clamped to 117, got {q_clamped[1]}"
    assert was_clamped[0] and was_clamped[1], "J1 and J2 should be flagged"
    assert not was_clamped[2] and not was_clamped[3], "J3 and J4 should not be flagged"
    print("PASS: test_clamp_above_max")


def test_clamp_below_min():
    """Angles below min should be clamped up."""
    q = [-100.0, 0.0, -150.0, 0.0]
    q_clamped, was_clamped = clamp_joints(q)
    assert q_clamped[0] == -90.0, f"J1 should be clamped to -90, got {q_clamped[0]}"
    assert q_clamped[2] == -117.0, f"J3 should be clamped to -117, got {q_clamped[2]}"
    print("PASS: test_clamp_below_min")


def test_validate_valid():
    """Valid angles should pass validation."""
    q = [45.0, -30.0, 60.0, -10.0]
    is_valid, violations = validate_joints(q)
    assert is_valid, f"Should be valid, got violations: {violations}"
    assert len(violations) == 0
    print("PASS: test_validate_valid")


def test_validate_invalid():
    """Invalid angles should be detected."""
    q = [91.0, 0.0, 0.0, -120.0]
    is_valid, violations = validate_joints(q)
    assert not is_valid, "Should be invalid"
    assert len(violations) == 2, f"Expected 2 violations, got {len(violations)}"
    assert violations[0]['joint'] == 1
    assert violations[1]['joint'] == 4
    print("PASS: test_validate_invalid")


def test_base_angle_front_half():
    """IK for targets in front half should have q1 within ±90°."""
    # Straight forward
    q = IK(200, 0, 100, 0)
    assert -90.0 <= q[0] <= 90.0, f"q1={q[0]} outside ±90° for forward target"
    
    # Far left (positive Y)
    q = IK(200, 200, 100, 0)
    assert -90.0 <= q[0] <= 90.0, f"q1={q[0]} outside ±90° for left target"
    
    # Far right (negative Y)
    q = IK(200, -200, 100, 0)
    assert -90.0 <= q[0] <= 90.0, f"q1={q[0]} outside ±90° for right target"
    print("PASS: test_base_angle_front_half")


def test_base_angle_behind_clamped():
    """IK for targets behind the arm should clamp q1 to ±90°."""
    # Target behind the arm (negative X, positive Y -> q1 would be > 90°)
    q = IK(-50, 200, 100, 0)
    assert -90.0 <= q[0] <= 90.0, f"q1={q[0]} should be clamped for behind target"
    print("PASS: test_base_angle_behind_clamped")


def test_round_trip_with_limits():
    """FK(IK(target)) should produce a reachable position (may differ from target if clamped)."""
    q = IK(200, 50, 100, 0)
    T_ee, _ = FK(q)
    pos = T_ee[:3, 3]
    
    # The result should be finite and reasonable
    assert all(np.isfinite(pos)), f"FK produced non-finite position: {pos}"
    print(f"  Round-trip position: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")
    print("PASS: test_round_trip_with_limits")


def test_get_limits():
    """get_limits should return a copy with correct structure."""
    limits = get_limits()
    assert 'min' in limits and 'max' in limits and 'names' in limits
    assert len(limits['min']) == 4
    assert len(limits['max']) == 4
    assert limits['min'][0] == -90.0  # Base limit
    assert limits['max'][0] == 90.0
    print("PASS: test_get_limits")


if __name__ == '__main__':
    tests = [
        test_clamp_within_limits,
        test_clamp_above_max,
        test_clamp_below_min,
        test_validate_valid,
        test_validate_invalid,
        test_base_angle_front_half,
        test_base_angle_behind_clamped,
        test_round_trip_with_limits,
        test_get_limits,
    ]
    
    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"FAIL: {test.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"ERROR: {test.__name__}: {e}")
            failed += 1
    
    print(f"\n{'='*40}")
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)}")
    if failed > 0:
        sys.exit(1)
