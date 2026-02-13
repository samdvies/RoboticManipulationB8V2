
import numpy as np

def get_dh():
    """
    Returns the Modified DH parameters for OpenManipulator-X
    dh_params = [a(i-1), alpha(i-1), d(i), theta_offset(i)]
    Units: mm, degrees
    """
    #           a(i-1)  alpha(i-1)  d(i)    theta_offset
    # Standard Straight-Link Model for Simulation
    # Matches the visualization meshes (robot_renderer.py) which are simple boxes/cylinders.
    # Link 1: Base to Shoulder (d=77)
    # Link 2: Shoulder to Elbow (a=128)
    # Link 3: Elbow to Wrist (a=124)
    # Link 4: Wrist to EE (a=126)
    
    dh_params = np.array([
        [0,      0,      77,     0],    # Joint 1
        [0,     -90,     0,     -90],   # Joint 2
        [128,    0,      0,      90],   # Joint 3
        [124,    0,      0,      0],    # Joint 4
        [126,    0,      0,      0]     # End Effector
    ], dtype=float)
    
    return dh_params
