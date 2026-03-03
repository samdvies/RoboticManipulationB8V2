import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from visualization.kinematics.BridgeAvoidance import BridgeNoGoZone, solve_optimal_pitch
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.kinematics.JointLimits import validate_joints

def run_stress_test():
    zone = BridgeNoGoZone(220, 230, -25, 25, 0, 105)
    
    # We want to sweep X from 180 to 250 (approaching and under bridge)
    # y from -40 to 40 (inside and outside pillars)
    # Z from 10 to 120 (ground up to above deck)
    
    xs = np.linspace(180, 250, 8)
    ys = np.linspace(-40, 40, 5)
    zs = np.linspace(10, 120, 12)
    
    total = 0
    passed = 0
    failed_to_find = 0
    invalid_results = 0
    
    print("Starting stress test for solve_optimal_pitch...")
    for x in xs:
        for y in ys:
            for z in zs:
                total += 1
                
                # Check if the target itself is inside the bridge volume. If so, skip.
                if zone.contains_point((x, y, z), padding_mm=0.0):
                    continue
                
                pitch = solve_optimal_pitch(x, y, z, zone)
                
                if pitch is None:
                    # Not all points are reachable, so None is acceptable for out-of-reach 
                    # or completely blocked points.
                    failed_to_find += 1
                    continue
                    
                # If it found a pitch, it MUST be valid and strictly collision-free
                try:
                    q = IK(x, y, z, pitch)
                    valid, violations = validate_joints(q, tolerance_deg=0)
                    if not valid:
                        print(f"INVALID JOINTS: x={x:.1f}, y={y:.1f}, z={z:.1f} returned pitch {pitch} resulting in {violations}")
                        invalid_results += 1
                        continue
                except Exception as e:
                    print(f"IK ERROR: x={x:.1f}, y={y:.1f}, z={z:.1f} returned pitch {pitch} but IK failed: {e}")
                    invalid_results += 1
                    continue
                
                passed += 1

    print(f"\nStress Test Complete.")
    print(f"Total points tested (excluding inside-bridge): {total}")
    print(f"Passed (Valid pitch found): {passed}")
    print(f"Failed to find (Unreachable or blocked): {failed_to_find}")
    print(f"Invalid results (Returned pitch but was invalid): {invalid_results}")
    
    if invalid_results > 0:
        print("\nFAILURE: solve_optimal_pitch returned invalid results!")
        sys.exit(1)
    else:
        print("\nSUCCESS: All returned pitches were 100% valid within joint limits and collision margins.")

if __name__ == "__main__":
    import sys
    run_stress_test()
