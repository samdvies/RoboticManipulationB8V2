%% calibrate_bridge_z.m
% Interactive script to jog the robot arm down to touch the bridge deck.
% This will give you the exact physical value for BRIDGE_Z_MAX.

clc; clear;
addpath(genpath('../src'));

PORT = 'COM4';
BAUD = 1000000;
VELOCITY = 10;   % Slow, safe jogging velocity
MODE = 2;        % Task-space linear

START_X = 225.0; % Center of bridge
START_Y = 0.0;
START_Z = 120.0; % Safely high above bridge
PITCH   = 0.0;   % Horizontal approach

fprintf('==========================================\n');
fprintf('  Bridge Z-Height Calibration\n');
fprintf('==========================================\n');

try
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    
    % Close gripper so tip is sharp/easy to measure with
    hw.closeGripper();
    pause(1);

    fprintf('Moving to safe start position above bridge...\n');
    hw.moveToPose(START_X, START_Y, START_Z, PITCH, 3.0, MODE, 10.0);
    pause(3.5);
    
    curr_z = START_Z;
    
    fprintf('\n--- Jog Controls ---\n');
    fprintf('  [w] +5mm (Up)\n');
    fprintf('  [s] -5mm (Down)\n');
    fprintf('  [e] +1mm (Up fine)\n');
    fprintf('  [x] -1mm (Down fine)\n');
    fprintf('  [q] Quit and keep current Z as BRIDGE_Z_MAX\n');
    fprintf('--------------------\n');

    while true
        fprintf('\nCurrent Z: %.1f mm\n', curr_z);
        ch = input('Command: ', 's');
        
        if strcmpi(ch, 'q')
            break;
        end
        
        target_z = curr_z;
        if strcmpi(ch, 'w'), target_z = curr_z + 5; end
        if strcmpi(ch, 's'), target_z = curr_z - 5; end
        if strcmpi(ch, 'e'), target_z = curr_z + 1; end
        if strcmpi(ch, 'x'), target_z = curr_z - 1; end
        
        % Safety limits
        if target_z < 30
            fprintf('  Warning: Z < 30mm blocked for safety.\n');
            continue;
        end
        
        if target_z ~= curr_z
            fprintf('  Moving to Z = %.1f ...\n', target_z);
            % Use 0.5s for 1mm moves, 1.0s for 5mm moves so it interpolates smoothly
            time_sec = 0.5;
            if abs(target_z - curr_z) > 2, time_sec = 1.0; end
            
            % Override default z_floor (20) with 10 for calibration
            hw.moveToPose(START_X, START_Y, target_z, PITCH, time_sec, MODE, 10.0);
            pause(time_sec + 0.1);
            curr_z = target_z;
        end
    end
    
    fprintf('\n==========================================\n');
    fprintf('FINAL BRIDGE_Z_MAX = %.1f mm\n', curr_z);
    fprintf('==========================================\n');
    
    hw.disableTorque();
    hw.disconnect();
    
catch ME
    fprintf('\nERROR: %s\n', ME.message);
    try hw.disableTorque(); hw.disconnect(); catch, end
end
