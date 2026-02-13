
import numpy as np
from .DH import get_dh

def rot_x(alpha):
    alpha_rad = np.deg2rad(alpha)
    c = np.cos(alpha_rad)
    s = np.sin(alpha_rad)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ])

def trans_x(a):
    return np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def rot_z(theta):
    theta_rad = np.deg2rad(theta)
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def trans_z(d):
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

def get_transform(a, alpha, d, theta):
    """
    Computes the homogeneous transform for Modified DH parameters
    T(i-1, i) = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
    """
    T = rot_x(alpha) @ trans_x(a) @ rot_z(theta) @ trans_z(d)
    return T

def FK(joint_angles):
    """
    FK Forward Kinematics for OpenManipulator-X
    
    Input:
        joint_angles: List or array of 4 joint angles [q1, q2, q3, q4] in DEGREES
        
    Outputs:
        T_ee: 4x4 Homogeneous Transformation Matrix of the End Effector relative to Base.
        global_transforms: List of 4x4 matrices containing transforms of each link frame 
                           relative to the Base frame (0->1, 0->2, ..., 0->EE).
    """
    dh = get_dh()
    num_links = dh.shape[0]
    
    # Ensure input is array
    q = np.array(joint_angles)
    if q.shape[0] != 4:
        raise ValueError('FK requires 4 joint angles.')
        
    # Pad q with 0 for the fixed end-effector link (Row 5 of DH)
    q_full = np.append(q, 0)
    
    T_global = np.eye(4)
    global_transforms = []
    
    for i in range(num_links):
        a = dh[i, 0]
        alpha = dh[i, 1]
        d = dh[i, 2]
        theta_offset = dh[i, 3]
        
        theta = q_full[i] + theta_offset
        
        # Compute transform for this link
        T_local = get_transform(a, alpha, d, theta)
        
        # Update global transform
        T_global = T_global @ T_local
        
        # Store
        global_transforms.append(T_global)
        
    T_ee = T_global
    
    return T_ee, global_transforms
