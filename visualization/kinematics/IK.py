
import numpy as np
from .DH import get_dh
from .JointLimits import clamp_joints

def IK(x, y, z, pitch, method='elbow_up'):
    """
    IK Inverse Kinematics for OpenManipulator-X

    Slanted-link IK accounting for the 24mm forward offset in the
    shoulder-to-elbow link (L_prox = 130.23mm, beta = 10.62 deg).

    method: 'elbow_up' or 'elbow_down'
    """
    # DH Parameters (Lengths)
    dh = get_dh()
    d1     = dh[0, 2]  # 77    (Base height)
    L_prox = dh[2, 0]  # 130.23 (Shoulder to Elbow, slanted)
    L_dist = dh[3, 0]  # 124   (Elbow to Wrist)
    L_tool = dh[4, 0]  # 126   (Wrist to EE)

    # Beta: tilt angle of the shoulder link
    beta_deg = np.degrees(np.arctan2(24, 128))  # 10.62 degrees

    # --- Step 1: Base Angle (q1) ---
    q1 = np.rad2deg(np.arctan2(y, x))

    # --- Step 2: Planar Projection ---
    r = np.sqrt(x**2 + y**2)

    pitch_rad = np.deg2rad(pitch)
    r_wc = r - L_tool * np.cos(pitch_rad)
    z_wc = (z - d1) - L_tool * np.sin(pitch_rad)

    # Distance from Shoulder (Joint 2) to Wrist Center
    D = np.sqrt(r_wc**2 + z_wc**2)

    # Check reachability
    max_reach = L_prox + L_dist
    if D > max_reach:
        print('Warning: Target out of reach. Clamping to max extent.')
        ratio = max_reach / D
        r_wc = r_wc * ratio
        z_wc = z_wc * ratio
        D = max_reach

    # --- Step 3: Elbow Angle ---
    cos_alpha = (L_prox**2 + L_dist**2 - D**2) / (2 * L_prox * L_dist)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)

    # --- Step 4: Shoulder Angle ---
    beta_r = np.arctan2(z_wc, r_wc)
    cos_psi = (L_prox**2 + D**2 - L_dist**2) / (2 * L_prox * D)
    cos_psi = np.clip(cos_psi, -1.0, 1.0)
    psi = np.arccos(cos_psi)

    if method == 'elbow_up':
        q2_geom = beta_r + psi
        q3_geom = (np.pi - alpha)
    else:
        q2_geom = beta_r - psi
        q3_geom = -(np.pi - alpha)

    # --- Step 5: Map geometric angles to DH joint angles ---
    # With slanted-link theta offsets (-90+beta) and (90-beta):
    q2 = (90.0 - beta_deg) - np.rad2deg(q2_geom)
    q3 = np.rad2deg(q3_geom) - (90.0 - beta_deg)

    # --- Step 6: Wrist Angle (q4) ---
    # Pitch coupling: pitch = -(q2 + q3 + q4)
    q4 = -pitch - q2 - q3

    result = [float(q1), float(q2), float(q3), float(q4)]

    # Enforce joint limits for safe operation
    result, _ = clamp_joints(result)

    return result

