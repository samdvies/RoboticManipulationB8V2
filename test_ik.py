import sys
import numpy as np
from visualization.kinematics.IK import IK
from visualization.kinematics.JointLimits import validate_joints

poses = [
    ("mouth_start", [150.0, -100.0, 100.0, 0.0]),
    ("mouth_mid", [125.0, -125.0, 125.0, -40.0]),
    ("mouth_end", [125.0, -140.0, 120.0, -70.0])
]

for name, pose in poses:
    try:
        q = IK(*pose)
        valid, violations = validate_joints(q)
        print(f"{name} {pose}: IK Success, Valid limits: {valid}")
        if not valid:
            print(f"  Violations: {violations}")
    except Exception as e:
        print(f"{name} {pose}: IK Failed - {e}")
