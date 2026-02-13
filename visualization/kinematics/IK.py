
import numpy as np
from .DH import get_dh
from .JointLimits import clamp_joints

def IK(x, y, z, pitch, method='elbow_up'):
    """
    IK Inverse Kinematics for OpenManipulator-X
    ...
    method: 'elbow_up' or 'elbow_down'
    """
    # ... (same) ... but inside function:
    
    # DH Parameters (Lengths)
    dh = get_dh()
    d1 = dh[0, 2] # 77 (Base height)
    L2 = dh[2, 0] # 128 (Link 2)
    L3 = dh[3, 0] # 124 (Link 3)
    L4 = dh[4, 0] # 126 (Link 4)
    
    # --- Step 1: Base Angle (q1) ---
    q1 = np.rad2deg(np.arctan2(y, x))
    
    # --- Step 2: Planar Projection ---
    r = np.sqrt(x**2 + y**2)
    
    pitch_rad = np.deg2rad(pitch)
    r_wc = r - L4 * np.cos(pitch_rad)
    z_wc = (z - d1) - L4 * np.sin(pitch_rad)
    
    # Distance from Shoulder (Joint 2) to Wrist Center
    D = np.sqrt(r_wc**2 + z_wc**2)
    
    # Check reachability
    max_reach = L2 + L3
    if D > max_reach:
        print('Warning: Target out of reach. Clamping to max extent.')
        ratio = max_reach / D
        r_wc = r_wc * ratio
        z_wc = z_wc * ratio
        D = max_reach
         
    # --- Step 3: Elbow Angle (q3) ---
    cos_alpha = (L2**2 + L3**2 - D**2) / (2*L2*L3)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)
    
    # --- Step 4: Shoulder Angle (q2) ---
    beta = np.arctan2(z_wc, r_wc)
    cos_psi = (L2**2 + D**2 - L3**2) / (2*L2*D)
    cos_psi = np.clip(cos_psi, -1.0, 1.0)
    psi = np.arccos(cos_psi)
    
    if method == 'elbow_up':
        # Solution 1 (Elbow Up)
        q2_geom = beta + psi
        q3_geom = (np.pi - alpha)
    else:
        # Solution 2 (Elbow Down)
        q2_geom = beta - psi
        q3_geom = -(np.pi - alpha)
    
    # Mapping to Joint Angles with Baseline Offsets
    # Matches DH.py: theta2 = q2 - 90, theta3 = q3 + 90
    # Correction: q2 axis is inverted relative to geometric angle.
    # q2_geom=90 (Up) -> theta=-90 (Up) -> q2=0.
    # q2_geom=0 (Fwd) -> theta=0 (Fwd) -> q2=90.
    # Relationship: q2 = 90 - q2_geom
    q2 = 90.0 - np.rad2deg(q2_geom)
    q3 = np.rad2deg(q3_geom) - 90.0
    
    # Calculate q4 to maintain target pitch
    # Global Pitch = theta2 + theta3 + theta4
    # But q4 axis appears inverted relative to pitch command:
    # Negative q4 -> Moves Up. Positive q4 -> Moves Down.
    # So to achieve Negative Pitch (Down), we need Positive q4 change.
    # q4 = -Pitch - q2 - q3
    q4 = -pitch - q2 - q3
    
    result = [float(q1), float(q2), float(q3), float(q4)]
    
    # Enforce joint limits for safe operation
    result, _ = clamp_joints(result)
    
    return result
