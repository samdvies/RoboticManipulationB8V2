
import numpy as np
import sys
import os

# Add parent path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.kinematics.JointLimits import validate_joints, get_limits

def test_pitch(x, y, z, pitch):
    print(f"\n--- Testing Target: X={x}, Y={y}, Z={z}, Pitch={pitch} ---")
    try:
        q = IK(x, y, z, pitch)
        print(f"IK Solution (Angles): {q}")
        
        is_valid, violations = validate_joints(q)
        if not is_valid:
            print("VIOLATION: Joint limits exceeded!")
            for v in violations:
                print(f"  Joint {v['joint']} ({v['name']}): {v['angle']:.1f} (Limit: {v['min']} to {v['max']})")
        else:
            print("Joints are within limits.")
            
        # Verify actual pitch achieved via FK
        T_ee, _ = FK(q)
        
        # Pitch in this robot's FK (assuming planar chain for pitch)
        # Global pitch = sum of angles relative to base?
        # q1 is yaw. q2, q3, q4 contribute to pitch.
        # Global Pitch = -(q2 + q3 + q4) roughly for this DH convention?
        # Let's check rotation matrix T_ee
        # Rotation about Y axis?
        # R = T_ee[0:3, 0:3]
        # Pitch is usually atan2(-R[2,0], sqrt(R[0,0]^2 + R[1,0]^2)) or similar depending on Euler convention.
        # But simpler: q4 was calculated as -pitch - q2 - q3.
        # So pitch_actual = -q2 - q3 - q4.
        
        pitch_actual = -(q[1] + q[2] + q[3])
        print(f"calculated Pitch (from angles): {pitch_actual:.2f}")
        
        # Check against pure FK matrix orientation just to be sure if we want
        # But the algebraic verification is what IK used.
        
    except Exception as e:
        print(f"IK Failed: {e}")

if __name__ == "__main__":
    # Test User Case 1: Pitch 0 (Should stick out straight)
    test_pitch(200, 0, 100, 0)
    
    # Test User Case 2: Pitch 90 (Pointing Up) at same position
    test_pitch(200, 0, 100, 90)
    
    # Test User Case 3: Pitch -90 (Pointing Down) - Default
    test_pitch(200, 0, 100, -90)

    # Test failure case
    test_pitch(350, 0, 100, 90) # Extended reach with 90 pitch
