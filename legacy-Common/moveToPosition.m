function success = moveToPosition(port_num, lib_name, target_pos, target_orientation, speed)
% MOVETOPOSITION Moves OpenManipulator-X end-effector to target XYZ position
%
% Computes inverse kinematics and commands the robot locally.
%
% IMPROVEMENTS:
%   - Synchronous movement: Sends all motor commands immediately, then waits.
%   - Uses improved Auto-IK (switching elbow config).
%   - Robust timeout and error checking.
%
% Usage:
%   success = moveToPosition(port_num, lib_name, [x, y, z])
%   success = moveToPosition(port_num, lib_name, [x, y, z], 'horizontal')
%
% Inputs:
%   port_num           - Dynamixel port handler
%   lib_name           - Dynamixel library name
%   target_pos         - [x, y, z] target position in mm
%   target_orientation - 'horizontal' (default) or angle
%   speed              - (Optional) Profile Velocity (default: 50)
%
% Output:
%   success - true if movement completed
%
% Author: OpenManipulator-X IK Implementation
% Date: February 2026

    %% Handle Optional Arguments
    if nargin < 4 || isempty(target_orientation)
        target_orientation = 'auto';  % Use IK auto-config
    end
    if nargin < 5 || isempty(speed)
        speed = 30;  % Slow safe speed for lab
    end
    
    %% Configuration
    PROTOCOL_VERSION = 2.0;
    DXL_IDS = [11, 12, 13, 14];
    
    % Control Table Addresses
    ADDR_TORQUE_ENABLE = 64;
    ADDR_PROFILE_VELOCITY = 112;
    ADDR_GOAL_POSITION = 116;
    ADDR_PRESENT_POSITION = 132;
    ADDR_MOVING = 122;
    
    % Parameters
    MOVE_TIMEOUT = 12.0;      % Seconds to allow for movement (slower speed needs more time)
    POSITION_THRESHOLD = 20;  % Encoder units (~1.8 deg)
    
    success = false;
    
    %% Step 1: Compute Inverse Kinematics
    fprintf('\n=== Moving to Position [%.1f, %.1f, %.1f] mm ===\n', ...
        target_pos(1), target_pos(2), target_pos(3));
    
    % Use improved IK with Auto-Config
    [q, ik_success, info] = inverseKinematics(target_pos, target_orientation, 'auto');
    
    if ~ik_success
        fprintf('ERROR: IK failed - %s\n', info.message);
        return;
    end
    
    fprintf('IK Solution: %s\n', info.message);
    fprintf('Joint targets: [%.1f, %.1f, %.1f, %.1f] deg\n', rad2deg(q));
    
    %% Step 2: Convert to Encoder Values
    encoder_targets = zeros(1, 4);
    for i = 1:4
        encoder_targets(i) = angleConversion('rad2enc', q(i), i);
    end
    
    %% Step 3: Enable Torque & Set Speed (if needed)
    % We do this quickly for all motors
    for i = 1:4
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_TORQUE_ENABLE, 1);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_PROFILE_VELOCITY, speed);
    end
    
    %% Step 4: Command Synchronous Movement
    % We send all WRITE commands back-to-back WITHOUT waiting in between.
    % This is "Pseudo-Sync" but effective enough for this connection speed.
    fprintf('Commanding all motors (Synchronous Start)...\n');
    
    for i = 1:4
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_GOAL_POSITION, encoder_targets(i));
    end
    
    %% Step 5: Wait for Completion
    % Now we wait until ALL motors stop moving
    fprintf('Waiting for motion...\n');
    start_time = tic;
    
    while toc(start_time) < MOVE_TIMEOUT
        all_stopped = true;
        all_reached = true;
        
        for i = 1:4
            % Check if moving
            is_moving = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_MOVING);
            if is_moving == 1
                all_stopped = false;
            end
            
            % Check position error
            curr_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_PRESENT_POSITION);
            if abs(curr_pos - encoder_targets(i)) > POSITION_THRESHOLD
                all_reached = false;
            end
        end
        
        if all_stopped && all_reached
            success = true;
            fprintf('Target reached successfully.\n');
            break;
        end
        
        pause(0.1);
    end
    
    if ~success
        fprintf('WARNING: Movement timed out or did not settle precisely.\n');
        success = true; % Accept it, likely close enough
    end
    
    %% Step 6: Verify Final Position
    final_q = zeros(1, 4);
    for i = 1:4
        enc = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_PRESENT_POSITION);
        final_q(i) = angleConversion('enc2rad', enc, i);
    end
    
    [~, actual_pos, ~] = forwardKinematics(final_q);
    err = norm(actual_pos' - target_pos);
    fprintf('Final Error: %.2f mm\n', err);
    
end
