
import numpy as np
from .DH import get_dh
from .FK import get_transform

def get_jacobian(joint_angles):
    """
    Computes the geometric Jacobian for OpenManipulator-X
    J = get_jacobian(q)

    Input:
        joint_angles: List or array of 4 joint angles [q1, q2, q3, q4] in DEGREES

    Output:
        J: 6x4 Jacobian Matrix
           Rows 0-2: Linear Velocity Jacobian (Jv)
           Rows 3-5: Angular Velocity Jacobian (Jw)

           v = J @ q_dot

    Note: The Jacobian is computed with respect to the Base Frame (0).
    """
    # Load DH Parameters
    dh = get_dh()
    
    # Number of active joints (4 joints)
    n_joints = 4
    
    # Ensure input is array
    q = np.array(joint_angles, dtype=float)
    
    # Pad input to match DH length if needed or just use 4
    # The MATLAB version iterates carefully. Let's match it.
    
    # Initialize global transform
    T_curr = np.eye(4)
    
    # Store axes (z) and positions (p) for the Jacobian
    z_axes = np.zeros((3, n_joints))
    p_axes = np.zeros((3, n_joints))
    
    # End Effector Position will be computed at the end
    
    # Iterate through links to build up transforms
    # We need z_(i-1) and p_(i-1) relative to base frame
    # For Modified DH:
    # The axis of rotation for joint i is Z_(i-1), but wait.
    # Modified DH uses proximal notation: Link i frame is at Joint i.
    # The parameter `alpha` rotates X_(i-1) to align Z_(i-1) with Z_i (roughly).
    # `theta` rotates about Z_i.
    
    # Let's look at `GetJacobian.m` logic again:
    # T_pre_theta = T_curr * RotX(alpha) * TransX(a)
    # z_axis = T_pre_theta(1:3, 3) (The Z axis of this new frame)
    # p_axis = T_pre_theta(1:3, 4) (The Origin of this new frame)
    
    # The logic in MATLAB was:
    # for i = 1:5
    #   ...
    #   T_pre_theta = T_curr * OpenManipulator.GetTransform(0, alpha, 0, 0) * OpenManipulator.GetTransform(a, 0, 0, 0); 
    #   z_axes(:, i) = T_pre_theta(1:3, 3);
    #   ...
    #   T_link = OpenManipulator.GetTransform(a, alpha, d, theta);
    #   T_curr = T_curr * T_link;
    
    # Wait, my `get_transform` from `FK.py` does:
    # RotX(alpha) * TransX(a) * RotZ(theta) * TransZ(d)
    
    # In MATLAB:
    # GetTransform(0, alpha, 0, 0) -> RotX(alpha) * ... = RotX(alpha)
    # GetTransform(a, 0, 0, 0) -> ... TransX(a) ... = TransX(a)
    
    # So `T_pre_theta` effectively computes the frame transformation *before* the joint rotation `theta`.
    # This aligns the Z-axis vector with the joint axis.
    
    for i in range(5):
        # DH params for row i (Link i)
        a = dh[i, 0]
        alpha = dh[i, 1]
        d = dh[i, 2]
        theta_offset = dh[i, 3]
        
        # Determine q_val
        if i < n_joints:
             q_val = q[i]
        else:
             q_val = 0.0 # Fixed EE
             
        theta = q_val + theta_offset
        
        # 1. Compute T_pre_theta to extract Z-axis and Position of the joint
        # This frame represents the coordinate system where Z is the axis of rotation for Joint i.
        # We use helper functions from FK logic if possible, or just build it manually to match MATLAB.
        # MATLAB: T_pre = T_curr * RotX(alpha) * TransX(a)
        
        # Let's construct RotX(alpha) * TransX(a) manually using get_transform?
        # get_transform(a, alpha, d=0, theta=0) 
        # = RotX(alpha) * TransX(a) * RotZ(0) * TransZ(0)
        # = RotX(alpha) * TransX(a) * I * I
        # = RotX(alpha) * TransX(a).
        # Perfect.
        
        T_partial = get_transform(a, alpha, 0, 0)
        T_pre_theta = T_curr @ T_partial
        
        if i < n_joints:
            z_axes[:, i] = T_pre_theta[0:3, 2] # Z vector (3rd column)
            p_axes[:, i] = T_pre_theta[0:3, 3] # Position vector (4th column)
            
        # 2. Update T_curr with the full transform including theta
        T_link = get_transform(a, alpha, d, theta)
        T_curr = T_curr @ T_link
        
    # End Effector Position (from final T_curr)
    p_e = T_curr[0:3, 3]
    
    # Build Jacobian
    J = np.zeros((6, n_joints))
    
    for i in range(n_joints):
        z_curr = z_axes[:, i]
        p_curr = p_axes[:, i]
        
        # Linear Velocity Jacobian: Jv = z_i x (p_e - p_i)
        # np.cross returns vector
        J[0:3, i] = np.cross(z_curr, (p_e - p_curr))
        
        # Angular Velocity Jacobian: Jw = z_i
        J[3:6, i] = z_curr
        
    return J
