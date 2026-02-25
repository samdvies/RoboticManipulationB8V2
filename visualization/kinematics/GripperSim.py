"""
GripperSim — Simulated gripper for OpenManipulator-X

Hardware-matched constants from HardwareInterface.m (test_pick_and_place.m workflow).

Control Modes:
    1. Position Control  — set_position_pct(pct)
    2. Binary Open/Close — open() / close()
    3. Object-Aware Grip — grip_object(width_mm)
    4. Force-Limited Grip — grip_force(max_force_pct)
"""

import numpy as np


class GripperSim:
    """Simulated gripper matching MATLAB HardwareInterface.m encoder scheme."""

    # ── Hardware Constants (from HardwareInterface.m) ──────────────────
    GRIPPER_ID = 15
    GRIPPER_OPEN_ENC  = 1365   # 120°  — fully open
    GRIPPER_CLOSE_ENC = 2276   # 200°  — fully closed (cube pickup)
    ENCODER_PER_DEG   = 4096 / 360  # ~11.378 enc/deg

    # Physical jaw geometry (approximate for XM430 gripper)
    JAW_MAX_WIDTH_MM  = 40.0   # fully open jaw gap
    JAW_MIN_WIDTH_MM  =  0.0   # fully closed

    # Force simulation (maps to DYNAMIXEL present_current concept)
    MAX_SIMULATED_FORCE = 100.0  # arbitrary % scale

    def __init__(self):
        self._encoder   = self.GRIPPER_OPEN_ENC  # start open
        self._force_pct = 0.0
        self._mode      = 'position'  # position | object | force
        self._object_present = False
        self._object_width_mm = 0.0

    # ── Encoder / Percentage Helpers ──────────────────────────────────

    @staticmethod
    def enc_to_pct(enc: float) -> float:
        """Encoder value → 0-100 % (0 = open, 100 = closed).
        Matches HardwareInterface.readGripperPosition()."""
        rng = GripperSim.GRIPPER_CLOSE_ENC - GripperSim.GRIPPER_OPEN_ENC
        pct = (enc - GripperSim.GRIPPER_OPEN_ENC) / rng * 100.0
        return float(np.clip(pct, 0.0, 100.0))

    @staticmethod
    def pct_to_enc(pct: float) -> int:
        """Percentage → encoder value.
        Matches HardwareInterface.setGripperPosition()."""
        pct = float(np.clip(pct, 0.0, 100.0))
        enc = GripperSim.GRIPPER_OPEN_ENC + \
              (pct / 100.0) * (GripperSim.GRIPPER_CLOSE_ENC - GripperSim.GRIPPER_OPEN_ENC)
        return int(round(enc))

    @staticmethod
    def pct_to_jaw_mm(pct: float) -> float:
        """Percentage (0=open,100=closed) → jaw gap in mm."""
        pct = float(np.clip(pct, 0.0, 100.0))
        return GripperSim.JAW_MAX_WIDTH_MM * (1.0 - pct / 100.0)

    @staticmethod
    def jaw_mm_to_pct(jaw_mm: float) -> float:
        """Jaw gap mm → percentage."""
        jaw_mm = float(np.clip(jaw_mm, GripperSim.JAW_MIN_WIDTH_MM,
                               GripperSim.JAW_MAX_WIDTH_MM))
        return (1.0 - jaw_mm / GripperSim.JAW_MAX_WIDTH_MM) * 100.0

    # ── Properties ────────────────────────────────────────────────────

    @property
    def pct(self) -> float:
        return self.enc_to_pct(self._encoder)

    @property
    def encoder(self) -> int:
        return self._encoder

    @property
    def jaw_width_mm(self) -> float:
        return self.pct_to_jaw_mm(self.pct)

    @property
    def force(self) -> float:
        return self._force_pct

    @property
    def mode(self) -> str:
        return self._mode

    # ── Mode 1 & 2: Position / Binary ─────────────────────────────────

    def set_position_pct(self, pct: float):
        """Set gripper to a percentage position (0=open, 100=closed)."""
        self._mode = 'position'
        pct = float(np.clip(pct, 0.0, 100.0))
        self._encoder = self.pct_to_enc(pct)
        self._force_pct = 0.0
        self._object_present = False

    def set_jaw_width_mm(self, jaw_width_mm: float):
        """Set gripper by physical jaw gap (mm)."""
        target_pct = self.jaw_mm_to_pct(jaw_width_mm)
        self.set_position_pct(target_pct)

    def open(self):
        """Fully open the gripper. Mirrors HardwareInterface.openGripper()."""
        self.set_position_pct(0.0)

    def close(self):
        """Fully close the gripper. Mirrors HardwareInterface.closeGripper()."""
        self.set_position_pct(100.0)

    # ── Mode 3: Object-Aware Grip ─────────────────────────────────────

    # Preset object widths (mm)
    PRESETS = {
        'cube_25mm': 25.0,
        'cylinder_20mm': 20.0,
        'sphere_30mm': 30.0,
        'pen': 12.0,
    }

    def grip_object(self, width_mm: float, squeeze_mm: float = 2.0):
        """Close gripper to match object width with optional squeeze.

        Args:
            width_mm:   Object diameter / width in mm.
            squeeze_mm: Extra squeeze beyond contact (simulates grip force).
        """
        self._mode = 'object'
        target_gap = max(0.0, width_mm - squeeze_mm)
        target_pct = self.jaw_mm_to_pct(target_gap)
        self._encoder = self.pct_to_enc(target_pct)
        self._object_present = True
        self._object_width_mm = width_mm

        # Simulated contact force proportional to squeeze
        if squeeze_mm > 0 and width_mm > 0:
            self._force_pct = min(100.0, (squeeze_mm / width_mm) * 100.0)
        else:
            self._force_pct = 0.0

    def grip_preset(self, name: str, squeeze_mm: float = 2.0):
        """Grip a named preset object."""
        if name not in self.PRESETS:
            raise ValueError(f"Unknown preset '{name}'. "
                             f"Available: {list(self.PRESETS.keys())}")
        self.grip_object(self.PRESETS[name], squeeze_mm)

    # ── Mode 4: Force-Limited Grip ────────────────────────────────────

    def grip_force(self, max_force_pct: float = 50.0):
        """Close gripper until simulated force reaches threshold.

        In hardware this maps to monitoring ADDR_PRESENT_CURRENT.
        In simulation, we model force as proportional to closure
        beyond a soft contact point.

        Args:
            max_force_pct: Stop when force reaches this % of max (0-100).
        """
        self._mode = 'force'
        # Simulate: force builds linearly from 70% closed onwards
        # At 70% closed → force = 0%
        # At 100% closed → force = 100%
        # So target_pct = 70 + (max_force_pct / 100) * 30
        CONTACT_PCT = 70.0  # gripper % where simulated contact begins
        FORCE_RANGE_PCT = 100.0 - CONTACT_PCT  # 30% of travel produces force

        target_pct = CONTACT_PCT + (max_force_pct / 100.0) * FORCE_RANGE_PCT
        target_pct = float(np.clip(target_pct, 0.0, 100.0))

        self._encoder = self.pct_to_enc(target_pct)
        self._force_pct = float(np.clip(max_force_pct, 0.0, 100.0))
        self._object_present = True

    # ── State ─────────────────────────────────────────────────────────

    def get_state(self) -> dict:
        """Return full gripper state dict."""
        return {
            'mode':           self._mode,
            'pct':            round(self.pct, 1),
            'encoder':        self._encoder,
            'jaw_width_mm':   round(self.jaw_width_mm, 2),
            'force_pct':      round(self._force_pct, 1),
            'object_present': self._object_present,
            'object_width_mm': round(self._object_width_mm, 2),
        }

    def __repr__(self):
        s = self.get_state()
        return (f"GripperSim(mode={s['mode']}, pct={s['pct']}%, "
                f"jaw={s['jaw_width_mm']}mm, force={s['force_pct']}%)")
