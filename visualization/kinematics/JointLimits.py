import numpy as np

# Joint limits in DEGREES for OpenManipulator-X (4 joints)
# Joint 1 (Base): +/-90 deg - user restriction: front half-plane only
# Joint 2 (Shoulder): +/-117 deg - conservative safe default
# Joint 3 (Elbow): +/-117 deg - conservative safe default
# Joint 4 (Wrist): +/-117 deg - conservative safe default

JOINT_LIMITS = {
    'min': np.array([-90.0, -117.0, -117.0, -117.0]),
    'max': np.array([90.0, 117.0, 117.0, 117.0]),
}

JOINT_NAMES = ['Base', 'Shoulder', 'Elbow', 'Wrist']


def get_limits():
    """Returns a copy of the joint limits dict."""
    return {
        'min': JOINT_LIMITS['min'].copy(),
        'max': JOINT_LIMITS['max'].copy(),
        'names': JOINT_NAMES.copy(),
    }


def clamp_joints(q, warn: bool = True, min_delta_warn_deg: float = 0.05):
    """
    Clamp joint angles to their limits.

    Args:
        q: list or array of 4 joint angles in degrees [q1, q2, q3, q4]
        warn: print clamp diagnostics
        min_delta_warn_deg: suppress tiny clamp deltas below this threshold

    Returns:
        q_clamped: numpy array of clamped joint angles (degrees)
        was_clamped: boolean array indicating which joints were clamped
    """
    q = np.array(q, dtype=float)
    q_clamped = np.clip(q, JOINT_LIMITS['min'], JOINT_LIMITS['max'])
    was_clamped = ~np.isclose(q, q_clamped)
    min_delta = max(0.0, float(min_delta_warn_deg))

    for i in range(len(q)):
        if was_clamped[i]:
            delta = abs(float(q[i]) - float(q_clamped[i]))
            if warn and delta >= min_delta:
                print(
                    f"Warning: Joint {i+1} ({JOINT_NAMES[i]}) clamped: "
                    f"{q[i]:.3f} deg -> {q_clamped[i]:.3f} deg "
                    f"(delta={delta:.3f}, limit=[{JOINT_LIMITS['min'][i]:.1f}, {JOINT_LIMITS['max'][i]:.1f}] deg)"
                )

    return q_clamped.tolist(), was_clamped.tolist()


def validate_joints(q, tolerance_deg: float = 0.5):
    """
    Validate joint angles against limits without modifying them.
    Joints within tolerance_deg of the limit are also flagged as violations,
    to catch IK-clamped solutions that are at the edge of the workspace.

    Args:
        q: list or array of 4 joint angles in degrees
        tolerance_deg: margin in degrees applied to each limit (default 0.5)

    Returns:
        is_valid: True if all joints are within limits (with tolerance margin)
        violations: list of dicts for each violating joint
    """
    q = np.array(q, dtype=float)
    tol = float(tolerance_deg)
    violations = []

    for i in range(len(q)):
        if q[i] <= JOINT_LIMITS['min'][i] + tol or q[i] >= JOINT_LIMITS['max'][i] - tol:
            violations.append({
                'joint': i + 1,
                'name': JOINT_NAMES[i],
                'angle': float(q[i]),
                'min': float(JOINT_LIMITS['min'][i]),
                'max': float(JOINT_LIMITS['max'][i]),
            })

    return len(violations) == 0, violations
