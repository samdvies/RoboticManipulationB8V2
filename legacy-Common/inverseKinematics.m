function [q, success, info] = inverseKinematics(target_pos, target_orientation, elbow_config)
% INVERSEKINEMATICS Computes joint angles for OpenManipulator-X to reach target position
%
% Geometric inverse kinematics solution for 4-DOF arm.
%
% IMPROVED IMPLEMENTATION:
%   - Uses official asymmetric joint limits (ROBOTIS e-Manual)
%   - Checks for ground collisions (Elbow hitting table)
%   - Automatic configuration switching (Elbow-Up <-> Elbow-Down)
%
% Usage:
%   [q, success, info] = inverseKinematics([x, y, z])
%   [q, success, info] = inverseKinematics([x, y, z], 'horizontal')
%   [q, success, info] = inverseKinematics([x, y, z], 'horizontal', 'auto')
%
% Inputs:
%   target_pos         - [x, y, z] target position in mm
%   target_orientation - 'horizontal' (default), 'down', 'forward', or angle in rad
%   elbow_config       - 'auto' (default), 'elbow-down', or 'elbow-up'
%
% Outputs:
%   q       - [q1, q2, q3, q4] joint angles in radians
%   success - true if solution found within limits and safe
%   info    - Structure with solution details
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

    %% Robot Physical Parameters
    L_base = 77;                              % Height from base to joint 2
    L_prox_x = 24;                            % Horizontal offset of link 2
    L_prox_z = 128;                           % Vertical component of link 2
    L_prox = sqrt(L_prox_x^2 + L_prox_z^2);   % Diagonal length ≈ 130.23 mm
    L_dist = 124;                             % Link 3 length (joint 3 to 4)
    L_tool = 126;                             % Tool length (joint 4 to end-effector)
    
    % Mechanical offset angle
    beta = atan2(L_prox_x, L_prox_z);         % ≈ 0.1855 rad (10.62°)
    
    %% Official Joint Limits (ROBOTIS e-Manual)
    % Format: [min, max] in radians
    LIMITS = [
        -pi/2,  pi/2;   % Joint 1 (Base): ±90° (User restricted)
        -2.05,  1.57;   % Joint 2 (Shoulder): -117° to +90°
        -1.57,  1.53;   % Joint 3 (Elbow): -90° to +87°
        -1.80,  2.00    % Joint 4 (Wrist): -103° to +114°
    ];
    
    %% Handle Optional Arguments
    if nargin < 2 || isempty(target_orientation)
        target_orientation = 'horizontal';
    end
    if nargin < 3 || isempty(elbow_config)
        elbow_config = 'auto';
    end
    
    %% Wrapper for Auto-Configuration
    % If 'auto', intelligently select initial configuration based on height
    if strcmpi(elbow_config, 'auto')
        % Heuristic: Low targets likely need Elbow-Up to avoid table collision
        if target_pos(3) < 50
            primary_config = 'elbow-up';
            secondary_config = 'elbow-down';
        else
            primary_config = 'elbow-down';
            secondary_config = 'elbow-up';
        end
        
        % Try Primary Config
        [q, success, info] = solveIK(target_pos, target_orientation, primary_config, L_base, L_prox, L_dist, L_tool, beta, LIMITS);
        
        if ~success
            % If failed, try Secondary
            [q_sec, success_sec, info_sec] = solveIK(target_pos, target_orientation, secondary_config, L_base, L_prox, L_dist, L_tool, beta, LIMITS);
            
            % If Secondary worked, or if both failed but Secondary was "better" (e.g. valid but collision), take it
            if success_sec || (info_sec.within_limits && ~info_sec.ground_collision)
                q = q_sec;
                success = success_sec;
                info = info_sec;
                info.message = ['(Auto-Switched to ' secondary_config ') ' info.message];
            end
        end
    else
        % Explicit configuration requested
        [q, success, info] = solveIK(target_pos, target_orientation, elbow_config, L_base, L_prox, L_dist, L_tool, beta, LIMITS);
    end
end

function [q, success, info] = solveIK(target_pos, target_orientation, elbow_config, L_base, L_prox, L_dist, L_tool, beta, LIMITS)
    %% Extract Target
    x = target_pos(1);
    y = target_pos(2);
    z = target_pos(3);
    
    %% Initialize Output
    q = [0, 0, 0, 0];
    success = false;
    info = struct();
    info.reachable = false;
    info.within_limits = false;
    info.ground_collision = false;
    info.config = elbow_config;
    info.fk_error = Inf;
    info.message = '';
    
    %% Step 1: Base Angle (q1)
    q1 = atan2(y, x);
    
    %% Step 2: Reduce to 2D Planar Problem
    r = sqrt(x^2 + y^2);
    z_rel = z - L_base;
    
    %% Step 3: Determine Wrist Position
    if ischar(target_orientation)
        switch lower(target_orientation)
            case 'horizontal'
                r_wrist = r - L_tool;
                z_wrist = z_rel;
                pitch_target = 0;
            case 'down'
                r_wrist = r;
                z_wrist = z_rel + L_tool;
                pitch_target = -pi/2;
            case 'forward'
                r_wrist = r - L_tool;
                z_wrist = z_rel;
                pitch_target = 0;
            otherwise % Treat as horizontal
                r_wrist = r - L_tool;
                z_wrist = z_rel;
                pitch_target = 0;
        end
    else
        pitch_target = target_orientation;
        r_wrist = r - L_tool * cos(pitch_target);
        z_wrist = z_rel - L_tool * sin(pitch_target);
    end
    
    %% Step 4: Solve 2-Link Planar Arm
    D_sq = r_wrist^2 + z_wrist^2;
    D_reach = sqrt(D_sq);
    
    max_reach = L_prox + L_dist;
    min_reach = abs(L_prox - L_dist); % Due to offset
    
    if D_reach > max_reach + 1 % 1mm tolerance
        info.message = sprintf('Unreachable: Distance %.1f > Max %.1f', D_reach, max_reach);
        return;
    end
    
    info.reachable = true;
    
    % Law of cosines
    cos_q3 = (D_sq - L_prox^2 - L_dist^2) / (2 * L_prox * L_dist);
    cos_q3 = max(-1, min(1, cos_q3)); % Clamp
    
    if strcmpi(elbow_config, 'elbow-up')
        q3_raw = -atan2(sqrt(1 - cos_q3^2), cos_q3);
    else % elbow-down
        q3_raw = atan2(sqrt(1 - cos_q3^2), cos_q3);
    end
    
    %% Step 5: Shoulder Angle (q2)
    phi = atan2(z_wrist, r_wrist);
    psi = atan2(L_dist * sin(q3_raw), L_prox + L_dist * cos(q3_raw));
    q2_raw = phi - psi;
    
    %% Step 6: Wrist Angle (q4)
    q4 = pitch_target - (q2_raw + q3_raw);
    
    %% Step 7: Apply Mechanical Offsets
    q2 = q2_raw - beta;
    q3 = q3_raw + beta;
    
    %% Step 8: Assemble & Limit Check
    q = [q1, q2, q3, q4];
    
    in_limits = true;
    bad_joint = 0;
    
    for i = 1:4
        if q(i) < LIMITS(i,1) - 0.01 || q(i) > LIMITS(i,2) + 0.01 % Small tolerance
            in_limits = false;
            bad_joint = i;
            break;
        end
    end
    
    info.within_limits = in_limits;
    
    if ~in_limits
        info.message = sprintf('Joint %d limit violation: %.1f deg (Range: %.1f to %.1f)', ...
            bad_joint, rad2deg(q(bad_joint)), rad2deg(LIMITS(bad_joint,1)), rad2deg(LIMITS(bad_joint,2)));
        return;
    end
    
    %% Step 9: Ground Collision Check
    % Calculate elbow height
    % Frame 2 is at L_base height.
    % Link 2 goes up/out.
    % Elbow Z (relative to base frame 0)
    % Z_elbow = L_base + L_prox * sin(q2 + beta) -- wait, check FK logic
    % FK: T_0_3 = T_0_2 * T_2_3.
    % T_0_2 puts us at shoulder.
    % Vector 2->3 is [L_prox, 0, 0] in frame 2.
    % Frame 2 is rotated by (q2 + beta) around Y axis... no, around Z axis of previous frame which was rotated...
    % Let's use simplified trig for Ground Check:
    % Shoulder is at Z = L_base.
    % Elbow is at Z = L_base + L_prox * sin(q2 + beta).
    % NOTE: In standard DH for this robot, q2=0 is horizontal.
    % Correct: z_elbow = L_base + L_prox * sin(q2_raw); where q2_raw = q2 + beta
    
    z_elbow = L_base + L_prox * sin(q2 + beta);
    
    if z_elbow < 15 % Safety buffer 15mm from table
        info.ground_collision = true;
        info.message = sprintf('Ground collision risk: Elbow Z = %.1f mm', z_elbow);
        return;
    end
    
    %% Step 10: FK Verification
    [~, fk_pos, ~] = forwardKinematics(q);
    info.fk_position = fk_pos';
    info.fk_error = norm(fk_pos' - target_pos);
    
    if info.fk_error > 5.0
        info.message = sprintf('High FK Error: %.1f mm (Singularity?)', info.fk_error);
        return; 
    end
    
    success = true;
    info.message = 'Solution found';
end
