"""
Unit tests for GripperSim — verifies encoder conversions, control modes,
and state consistency against MATLAB HardwareInterface.m constants.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from visualization.kinematics.GripperSim import GripperSim


def test_encoder_constants():
    """Encoder values must match HardwareInterface.m"""
    assert GripperSim.GRIPPER_OPEN_ENC == 1365
    assert GripperSim.GRIPPER_CLOSE_ENC == 2276
    assert GripperSim.GRIPPER_ID == 15
    print("  [PASS] encoder_constants")


def test_open_close():
    """open() → 0%, close() → 100%"""
    g = GripperSim()
    g.open()
    assert g.pct == 0.0
    assert g.encoder == 1365
    assert abs(g.jaw_width_mm - 40.0) < 0.1

    g.close()
    assert g.pct == 100.0
    assert g.encoder == 2276
    assert abs(g.jaw_width_mm - 0.0) < 0.1
    print("  [PASS] open_close")


def test_pct_to_enc_roundtrip():
    """Percentage ↔ encoder round-trips correctly."""
    for pct in [0, 25, 50, 75, 100]:
        enc = GripperSim.pct_to_enc(pct)
        pct_back = GripperSim.enc_to_pct(enc)
        assert abs(pct_back - pct) < 1.0, f"Roundtrip failed at {pct}%: got {pct_back}%"
    print("  [PASS] pct_to_enc_roundtrip")


def test_set_position_pct():
    """setGripperPosition(pct) — matches MATLAB formula."""
    g = GripperSim()
    g.set_position_pct(50)
    # Expected enc = 1365 + (50/100) * (2276-1365) = 1365 + 455.5 = 1820.5 → 1821
    assert abs(g.encoder - 1821) <= 1
    assert abs(g.pct - 50.0) < 1.0
    print("  [PASS] set_position_pct")


def test_grip_object_cube():
    """grip_object(25mm) closes to ~25mm gap minus squeeze."""
    g = GripperSim()
    g.grip_object(25.0, squeeze_mm=2.0)
    # Target gap = 25 - 2 = 23mm
    # Expected pct = (1 - 23/40) * 100 = 42.5%
    assert 40.0 < g.pct < 45.0, f"Expected ~42.5%, got {g.pct}%"
    assert g.jaw_width_mm > 20.0  # should be ~23mm
    assert g.force > 0.0  # squeeze produces force
    assert g.get_state()['object_present'] is True
    print("  [PASS] grip_object_cube")


def test_grip_force():
    """grip_force(50) locks at 50% simulated force."""
    g = GripperSim()
    g.grip_force(50.0)
    assert abs(g.force - 50.0) < 0.1
    # At 50% force: pct = 70 + (50/100)*30 = 85%
    assert 83.0 < g.pct < 87.0, f"Expected ~85%, got {g.pct}%"
    print("  [PASS] grip_force")


def test_get_state_keys():
    """get_state() must return all expected keys."""
    g = GripperSim()
    state = g.get_state()
    expected_keys = {'mode', 'pct', 'encoder', 'jaw_width_mm', 
                     'force_pct', 'object_present', 'object_width_mm'}
    assert set(state.keys()) == expected_keys, f"Missing keys: {expected_keys - set(state.keys())}"
    print("  [PASS] get_state_keys")


def test_preset():
    """grip_preset('cube_25mm') works."""
    g = GripperSim()
    g.grip_preset('cube_25mm')
    assert g.get_state()['object_present'] is True
    assert g.jaw_width_mm < 25.0  # squeezed
    print("  [PASS] preset")


if __name__ == '__main__':
    print("=== GripperSim Unit Tests ===")
    test_encoder_constants()
    test_open_close()
    test_pct_to_enc_roundtrip()
    test_set_position_pct()
    test_grip_object_cube()
    test_grip_force()
    test_get_state_keys()
    test_preset()
    print("\nAll tests passed!")
