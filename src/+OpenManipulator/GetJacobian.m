function J = GetJacobian(joint_angles)
%GETJACOBIAN Computes the geometric Jacobian for OpenManipulator-X
%   J = GetJacobian(q)
%
%   Input:
%       joint_angles: 1x4 vector [q1, q2, q3, q4] in DEGREES
%
%   Output:
%       J: 6x4 Jacobian Matrix
%          Rows 1-3: Linear Velocity Jacobian (Jv)
%          Rows 4-6: Angular Velocity Jacobian (Jw)
%
%          v = J * q_dot
%
%   Note: The Jacobian is computed with respect to the Base Frame (0).

    % Load DH Parameters
    dh = OpenManipulator.GetDH();
    
    % Number of active joints (4 joints, but 5 rows in DH table including EE)
    n_joints = 4;
    
    % Ensure input is row vector
    q = double(joint_angles(:)'); 
    
    % FK to get link transforms
    % We need transforms T01, T02, T03, T04, T0E
    % T0i = T01 * ... * T(i-1)i
    
    T_global = cell(1, n_joints + 1); % Store T01, T02, T03, T04, T0E
    T_curr = eye(4);
    
    % Pad q with 0 for the fixed end-effector link (Row 5 of DH)
    q_full = [q, 0];
    
    % Check OpenManipulator convention: 
    % T = RotX(alpha) * TransX(a) * RotZ(theta) * TransZ(d)
    
    T_curr = eye(4);
    
    % Store axes (z) and positions (p) for the Jacobian
    z_axes = zeros(3, n_joints);
    p_axes = zeros(3, n_joints);
    
    % Compute forward kinematics and extract Jacobian axes on the fly
    for i = 1:5
        a = dh(i, 1);
        alpha = dh(i, 2);
        d = dh(i, 3);
        theta_offset = dh(i, 4);
        
        % Joint angle is only relevant for the rotation part of this link
        % theta = q + offset
        if i <= n_joints
             q_val = q(i);
        else
             q_val = 0; % Fixed EE pose
        end
        theta = q_val + theta_offset;
        
        % In this specific DH convention (Modified DH):
        % The Joint i rotation (theta) happens AFTER RotX(alpha) and TransX(a).
        % So the axis of rotation for Joint i is the Z-axis of the frame
        % formulated by T_curr * RotX(alpha) * TransX(a).
        
        % Intermediate transform to the "Joint Axis Frame"
        T_pre_theta = T_curr * OpenManipulator.GetTransform(0, alpha, 0, 0) * OpenManipulator.GetTransform(a, 0, 0, 0); 
        % Note: Using GetTransform(0,alpha,0,0) is just RotX(alpha)
        
        if i <= n_joints
            z_axes(:, i) = T_pre_theta(1:3, 3);
            p_axes(:, i) = T_pre_theta(1:3, 4);
        end
        
        % Complete the transform for this link
        T_link = OpenManipulator.GetTransform(a, alpha, d, theta);
        T_curr = T_curr * T_link;
    end
    
    % End Effector Position
    p_e = T_curr(1:3, 4);
    
    % Build Jacobian
    J = zeros(6, n_joints);
    
    for i = 1:n_joints
        z_curr = z_axes(:, i);
        p_curr = p_axes(:, i);
        
        J(1:3, i) = cross(z_curr, (p_e - p_curr));
        J(4:6, i) = z_curr;
    end
    
end
