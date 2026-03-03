%% calibrate_bridge_x.m
% Interactive script to jog the robot arm forward to touch the bridge edge.
% This will give you the exact physical value for BRIDGE_X_MIN.

clc; clear;
addpath(genpath('../src'));

PORT = 'COM4';
BAUD = 1000000;
VELOCITY = 10;   % Slow, safe jogging velocity
MODE = 2;        % Task-space linear

% Ask user for the safe Z-height (should be based on what was found in Z calibration)
suggested_z = 70.0; % Something safely under the deck
fprintf('Enter safe Z-height to crawl under the bridge at (e.g. 70).\n');
user_z = input(sprintf('Z Height [%.1f]: ', suggested_z));
if isempty(user_z)
    START_Z = suggested_z;
else
    START_Z = user_z;
end

START_X = 150.0; % Comfortably far back from the bridge
START_Y = 0.0;
PITCH   = 0.0;   % Horizontal approach

fprintf('\n==========================================\n');
fprintf('  Bridge X-Edge Calibration\n');
fprintf('==========================================\n');

try
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    
    % Close gripper so it's a consistent shape
    hw.closeGripper();
    pause(1);

    fprintf('Moving to safe start position in front of bridge...\n');
    hw.moveToPose(START_X, START_Y, START_Z, PITCH, 3.0, MODE, 10.0);
    pause(3.5);
    
    curr_x = START_X;
    
    fprintf('\n--- Jog Controls ---\n');
    fprintf('  [w] +5mm (Forward into bridge)\n');
    fprintf('  [s] -5mm (Back away)\n');
    fprintf('  [e] +1mm (Forward fine)\n');
    fprintf('  [x] -1mm (Back fine)\n');
    fprintf('  [q] Quit and keep current X as BRIDGE_X_MIN\n');
    fprintf('--------------------\n');

    while true
        fprintf('\nCurrent X: %.1f mm\n', curr_x);
        ch = input('Command: ', 's');
        
        if strcmpi(ch, 'q')
            break;
        end
        
        target_x = curr_x;
        if strcmpi(ch, 'w'), target_x = curr_x + 5; end
        if strcmpi(ch, 's'), target_x = curr_x - 5; end
        if strcmpi(ch, 'e'), target_x = curr_x + 1; end
        if strcmpi(ch, 'x'), target_x = curr_x - 1; end
        
        % Safety limits
        if target_x > 250
            fprintf('  Warning: X > 250mm blocked for safety.\n');
            continue;
        end
        
        if target_x ~= curr_x
            fprintf('  Moving to X = %.1f ...\n', target_x);
            % Use 0.5s for 1mm moves, 1.0s for 5mm moves
            time_sec = 0.5;
            if abs(target_x - curr_x) > 2, time_sec = 1.0; end
            
            hw.moveToPose(target_x, START_Y, START_Z, PITCH, time_sec, MODE, 10.0);
            pause(time_sec + 0.1);
            curr_x = target_x;
        end
    end
    
    fprintf('\n==========================================\n');
    fprintf('FINAL BRIDGE_X_MIN = %.1f mm\n', curr_x);
    fprintf('==========================================\n');
    
    hw.disableTorque();
    hw.disconnect();
    
catch ME
    fprintf('\nERROR: %s\n', ME.message);
    try hw.disableTorque(); hw.disconnect(); catch, end
end
