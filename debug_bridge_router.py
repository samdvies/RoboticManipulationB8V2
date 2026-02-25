import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from visualization.kinematics.BridgeAvoidance import (
    BridgeNoGoZone, build_bridge_zones, plan_bridge_safe_waypoints,
    _check_path_segment
)
from visualization.kinematics.IK import IK
from visualization.kinematics.JointLimits import validate_joints

# Real bridge specs
zone = BridgeNoGoZone(
    x_min=187.5, x_max=212.5,
    y_min=-35.0, y_max=35.0,
    z_min=0.0,   z_max=60.0
)
zones = build_bridge_zones(zone, gap_y=50.0, pillar_width_y=10.0)

tests = [
    ("Pick (in)",  [200.0, 0.0, 180.0, 0.0], [200.0, 0.0, 30.0, 0.0]),
    ("Exit (out)", [200.0, 0.0, 30.0,  0.0], [200.0, 0.0, 180.0, 0.0]),
]

with open('debug_output.txt', 'w', encoding='utf-8') as f:
    f.write(f"Bridge bounding box: X=[{zone.x_min}, {zone.x_max}] Z=[{zone.z_min}, {zone.z_max}]\n")
    f.write(f"Solid zones: {len(zones)}\n")
    for i, z in enumerate(zones):
        f.write(f"  Zone {i}: X=[{z.x_min:.1f},{z.x_max:.1f}] Y=[{z.y_min:.1f},{z.y_max:.1f}] Z=[{z.z_min:.1f},{z.z_max:.1f}]\n")
    
    for test_name, start, target in tests:
        wps = plan_bridge_safe_waypoints(start, target, zone, zones=zones)
        f.write(f"\n{test_name}: {start} -> {target}\n")
        f.write(f"Waypoints ({len(wps)}):\n")
        for i, wp in enumerate(wps):
            try:
                q = IK(wp[0], wp[1], wp[2], wp[3])
                valid, _ = validate_joints(q, tolerance_deg=0)
                f.write(f"  {i}: [{wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f}, {wp[3]:.0f}] valid={valid} q=[{q[0]:.1f},{q[1]:.1f},{q[2]:.1f},{q[3]:.1f}]\n")
            except Exception as e:
                f.write(f"  {i}: [{wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f}, {wp[3]:.0f}] IK FAIL: {e}\n")
        
        path = [start] + wps
        all_ok = True
        for i in range(len(path) - 1):
            hit = _check_path_segment(path[i], path[i+1], zones)
            status = "FAIL" if hit else "OK"
            if hit:
                all_ok = False
            f.write(f"  Seg {i}: {status}\n")
        f.write(f"Result: {'ALL CLEAR' if all_ok else 'COLLISION DETECTED'}\n")
