
import sys
import os
import numpy as np

# Add src to path to allow imports if needed, but we are importing from visualization package
# We need to make sure we can import 'visualization'
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from visualization.kinematics.Jacobian import get_jacobian

def verify_jacobian():
    print("=== Jacobian Math Verification (Python) ===")
    
    # 1. Define known configuration matches MATLAB script
    q_home = [0, -45, 45, 0]
    print(f"Configuration: {q_home}")
    
    # 2. Compute Jacobian
    J = get_jacobian(q_home)
    
    print("Jacobian Matrix J:")
    # Format to match MATLAB output style roughly
    for row in J:
        print("   " + "   ".join([f"{x:.4f}" for x in row]))
        
    # 3. Check Dimensions
    if J.shape != (6, 4):
        print(f"Error: Dimensions {J.shape} != (6, 4)")
    else:
        print("Dimensions OK.")
        
    # 4. Check Singularity (Condition Number)
    # MATLAB: cond(J(1:3, :))
    # Python: default cond is 2-norm, same as MATLAB
    c = np.linalg.cond(J[0:3, :])
    print(f"Condition Number (Linear): {c:.4f}")
    
    if c > 100:
        print("[WARNING] Configuration is near singularity.")
    else:
        print("Configuration is well-conditioned.")
        
    # 5. Check Physical Consistency
    # Joint 1
    col1 = J[:, 0]
    print(f"Joint 1 Column: [{col1[0]:.2f}, {col1[1]:.2f}, {col1[2]:.2f}, {col1[3]:.2f}, {col1[4]:.2f}, {col1[5]:.2f}]")
    
    # Joint 2
    col2 = J[:, 1]
    print(f"Joint 2 Column: [{col2[0]:.2f}, {col2[1]:.2f}, {col2[2]:.2f}, {col2[3]:.2f}, {col2[4]:.2f}, {col2[5]:.2f}]")
    
    # Logic Checks
    if abs(col2[1]) < 1e-3 and (abs(col2[0]) > 1e-3 or abs(col2[2]) > 1e-3):
        print("Joint 2 logic checks out (rotates around Y, moves in X-Z).")
    else:
        print(f"[WARNING] Joint 2 logic UNEXPECTED. Vy={col2[1]:.2f} (Should be 0)")
        
    if abs(col1[0]) < 1e-6 and abs(col1[2]) < 1e-6 and abs(col1[5]) > 0.9:
        print("Joint 1 logic checks out (rotates around Z, moves in Y).")
    else:
        print("[WARNING] Joint 1 logic seems unexpected. Check frames.")
        
    print("Verification Complete.")

if __name__ == "__main__":
    verify_jacobian()
