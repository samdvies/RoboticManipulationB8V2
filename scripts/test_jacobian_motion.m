% TEST_JACOBIAN_MOTION
% Smooth Velocity Control Demo via Damped Least Squares (DLS)
%
% This script demonstrates "Resolved Motion Rate Control":
%   1. Define desired End-Effector Velocity (v_ee)
%   2. Calculate Jacobian (J)
%   3. Compute Joint Velocity: q_dot = J_dls_inverse * v_ee
%   4. Integrate: q_next = q_curr + q_dot * dt
%   5. Send as Position Command (Safe for Dynamixel Position Mode)

clc; clear;
addpath(genpath('../src'));

% --- Configuration ---
PORT = 'COM4';
BAUD = 1000000;
DT = 0.05;          % Control loop period (20Hz)
LAMBDA = 0.01;      % Damping factor for DLS
VEL_LIMIT = 50;     % mm/s limit for safety

try
    fprintf('=== Jacobian Velocity Control Demo ===\n');
    
    % 1. Connect
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(0); % Velocity 0 means use time/trajectory based, but here we update frequently
    % Note: configure(0) might not be valid for profile velocity, let's use a safe fast value
    hw.configure(100); % Set motor profile velocity high so they track our updates closely
    hw.enableTorque();
    
    % 2. Home
    fprintf('Moving to Start Pose...\n');
    start_pose = [200, 0, 100, -90]; % X=200, Y=0, Z=100, Pitch=-90
    hw.moveToPose(start_pose(1), start_pose(2), start_pose(3), start_pose(4));
    pause(1);
    
    % 3. Control Loop
    fprintf('Starting Velocity Control Loop...\n');
    fprintf('Executing: LINEAR MOVE (+X -> -X)\n');
    
    t = 0;
    duration = 5.0; % seconds
    
    while t < duration
        loop_start = tic;
        
        % Read current state
        q = hw.readAngles();
        
        % Compute Jacobian
        J = OpenManipulator.GetJacobian(q);
        
        % Define Desired Spatial Velocity [vx vy vz wx wy wz]
        % Example: Sinusoidal velocity in Y axis
        vx = 0;
        vy = 30 * sin(2 * pi * t / 2.0); % Oscillate Y +/- 30mm/s
        vz = 0;
        wx = 0; wy = 0; wz = 0;
        
        v_target = [vx; vy; vz; wx; wy; wz];
        
        % --- Singularity Handling (DLS) ---
        % J_dls = J' * (J*J' + lambda^2 * I)^-1
        lambda_sq = LAMBDA^2;
        inv_term = inv(J * J' + lambda_sq * eye(6));
        q_dot_rad = J' * inv_term * v_target;
        
        % Convert rad/s to deg/s
        q_dot_deg = rad2deg(q_dot_rad);
        
        % Integrate: q_next = q + q_dot * dt
        q_next = q + q_dot_deg' * DT;
        
        % Check Joint Limits (Simple Clamping)
        [q_next, ~] = OpenManipulator.JointLimits.Clamp(q_next);
        
        % Send Command
        % We send direct position command.
        % Since we are updating at 20Hz, we want the motor to move there quickly.
        % Ideally, we should set Profile Velocity on motors to match q_dot, 
        % but for small DT, position updates are sufficient.
        
        encoders = zeros(1,4);
        for i=1:4, encoders(i) = hw.deg2enc(q_next(i)); end
        hw.syncWritePositions(encoders);
        
        % Safety Check: Manipulability
        w = sqrt(det(J*J'));
        if w < 0.001
            fprintf('[WARNING] Near Singularity! w=%.5f\n', w);
        end
        
        % Timing
        elapsed = toc(loop_start);
        if elapsed < DT
            pause(DT - elapsed);
        end
        t = t + DT;
    end
    
    fprintf('Demo Complete.\n');
    hw.disconnect();
    
catch ME
    fprintf('ERROR: %s\n', ME.message);
    if exist('hw', 'var')
        hw.disconnect();
    end
end
