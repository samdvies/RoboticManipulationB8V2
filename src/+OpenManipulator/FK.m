function [T_ee, global_transforms] = FK(joint_angles)
%FK Forward Kinematics for OpenManipulator-X
%   [T_ee, global_transforms] = FK(joint_angles)
%
%   Input:
%       joint_angles: 1x4 or 4x1 vector of joint angles [q1, q2, q3, q4] in DEGREES
%
%   Outputs:
%       T_ee: 4x4 Homogeneous Transformation Matrix of the End Effector relative to Base.
%       global_transforms: 4x4xN matrix containing transforms of each link frame 
%                          relative to the Base frame (0->1, 0->2, ..., 0->EE).

    dh = OpenManipulator.GetDH();
num_links = size(dh, 1);

% Combine joint angles with offsets % dh column 4 is theta_offset.%
    joint_angles corresponds to the first 4 rows.The 5th row(EE) has 0 angle.

    % Ensure input is row vector q = joint_angles( :)'; if length (q) ~=
    4 error('FK requires 4 joint angles.');
    end
    
    % Pad q with 0 for the fixed end-effector link
    q_full = [q, 0];

    % Initialize transforms T_global = eye(4);
    global_transforms = zeros(4, 4, num_links);

    for
      i = 1 : num_links a = dh(i, 1);
    alpha = dh(i, 2);
    d = dh(i, 3);
    theta_offset = dh(i, 4);

    theta = q_full(i) + theta_offset;

        % Compute transform for this link
        T_local = OpenManipulator.GetTransform(a, alpha, d, theta);

        % Update global transform T_global = T_global * T_local;

        % Store global_transforms( :, :, i) = T_global;
        end

            T_ee = T_global;

        end
