
import numpy as np

def get_dh():
    """
    Returns the Modified DH parameters for OpenManipulator-X
    dh_params = [a(i-1), alpha(i-1), d(i), theta_offset(i)]
    Units: mm, degrees

    Slanted-link model: accounts for the physical 24mm forward offset
    in the shoulder-to-elbow link (Joint 2 to Joint 3).

    Physical geometry:
      Link 2-3: 128mm vertical + 24mm forward => L_prox = 130.23mm
      Tilt angle: beta = atan2(24, 128) = 10.62 degrees
    """
    L_prox_x = 24   # Forward offset (mm)
    L_prox_z = 128   # Vertical component (mm)
    L_prox = np.sqrt(L_prox_x**2 + L_prox_z**2)   # 130.23 mm
    beta_deg = np.degrees(np.arctan2(L_prox_x, L_prox_z))  # 10.62 deg

    dh_params = np.array([
        [0,        0,      77,     0],                 # Joint 1
        [0,       -90,     0,     -90 + beta_deg],     # Joint 2
        [L_prox,   0,      0,      90 - beta_deg],     # Joint 3
        [124,      0,      0,      0],                 # Joint 4
        [126,      0,      0,      0]                  # End Effector
    ], dtype=float)

    return dh_params

