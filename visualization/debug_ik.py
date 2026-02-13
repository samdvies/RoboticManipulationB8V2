
import sys
import os
import numpy as np

# Ensure we can import from the root directory
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK

def test_point(x, y, z):
    pitch = 0
    print(f"\n--- Testing Target: X={x}, Y={y}, Z={z}, Pitch={pitch} ---")
    
    # Solve IK
    try:
        q = IK(x, y, z, pitch)
        print(f"IK Resulting Joints (deg): {q}")
    except Exception as e:
        print(f"IK Failed: {e}")
        return

    # Check FK
    T_ee, _ = FK(q)
    pos_fk = T_ee[:3, 3]
    print(f"FK Result (pos): {pos_fk}")
    
    error = pos_fk - np.array([x, y, z])
    print(f"Error Vector [dx, dy, dz]: {error}")
    print(f"Norm Error: {np.linalg.norm(error)}")

def sweep_z():
    print("\n=== Sweeping Z (X=200, Y=0) ===")
    x = 200
    y = 0
    for z in range(50, 250, 50):
        test_point(x, y, z)

def check_q0():
    print("\n=== Checking Home Pose (q=[0,0,0,0]) ===")
    q = [0, 0, 0, 0]
    T_ee, _ = FK(q)
    print(f"FK Result (pos): {T_ee[:3, 3]}")

if __name__ == "__main__":
    check_q0()
    # Test specific point
    # test_point(200, 0, 100)
    
    # Run Sweep
    # sweep_z()
