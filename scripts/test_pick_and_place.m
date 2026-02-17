% TEST_PICK_AND_PLACE Interactive manipulation sequence
%
% Usage:
%   run('scripts/test_pick_and_place.m')
%
% Features:
%   - Interactive coordinate entry [x, y, z, pitch]
%   - "Pre-move" hover to verify position
%   - Confirmation loops (Retry if position is wrong)

clc; clear;
addpath(genpath('../src'));

% --- Configuration ---
PORT = 'COM4';
BAUD = 1000000;
VELOCITY = 30;
HOVER_OFFSET = 50; % mm above target

% Defaults
def_pick  = [200, 0, 50, -90];
def_place = [150, 150, 100, -90];
home_pose = [134, 0, 240, -45];

try
    fprintf('=== Interactive Pick & Place ===\n');
    
    % 1. Connect
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    
    % 2. Home
    fprintf('Moving Home...\n');
    hw.moveToAnglesInterpolated(OpenManipulator.IK(home_pose(1), home_pose(2), home_pose(3), home_pose(4)), 20);
    hw.openGripper();
    pause(1);

    % --- PICK PHASE ---
    while true
        fprintf('\n--- PICK CONFIGURATION ---\n');
        fprintf('Enter PICK pose [x y z pitch] (Default: [%g %g %g %g]): ', def_pick);
        user_in = input('');
        if isempty(user_in), pose = def_pick; else, pose = user_in; end
        def_pick = pose;
        
        fprintf('Moving to TARGET position (%.1f, %.1f, %.1f)...\n', ...
            pose(1), pose(2), pose(3));
        
        try
            % Use High-Level Move (Handles IK + Safety)
            hw.moveToPose(pose(1), pose(2), pose(3), pose(4));
        catch ME
            fprintf('Move Failed: %s\nTry again.\n', ME.message);
            continue;
        end
        
        % Confirmation
        choice = input('Correct position? Pick? (y/n/retry): ', 's');
        if strcmpi(choice, 'y')
            fprintf('Gripping...\n');
            hw.closeGripper();
            pause(1);
            
            fprintf('Lifting...\n');
            hw.moveToPose(pose(1), pose(2), pose(3)+100, pose(4));
            break; % Exit loop
        else
            fprintf('Retrying...\n');
        end
    end

    % --- PLACE PHASE ---
    while true
        fprintf('\n--- PLACE CONFIGURATION ---\n');
        fprintf('Enter PLACE pose [x y z pitch] (Default: [%g %g %g %g]): ', def_place);
        user_in = input('');
        if isempty(user_in), pose = def_place; else, pose = user_in; end
        def_place = pose;
        
        fprintf('Moving to TARGET position (%.1f, %.1f, %.1f)...\n', ...
            pose(1), pose(2), pose(3));
        
        try
            hw.moveToPose(pose(1), pose(2), pose(3), pose(4));
        catch ME
             fprintf('Move Failed: %s\nTry again.\n', ME.message);
             continue;
        end
        
        % Confirmation
        choice = input('Correct position? Place? (y/n/retry): ', 's');
        if strcmpi(choice, 'y')
            fprintf('Releasing...\n');
            hw.openGripper();
            pause(1);
            
            fprintf('Lifting...\n');
            hw.moveToPose(pose(1), pose(2), pose(3)+100, pose(4));
            break; % Exit loop
        else
            fprintf('Retrying...\n');
        end
    end
    
    % Return Home
    fprintf('\nSequence Complete. Returning Home.\n');
    hw.moveHome();
    hw.disconnect();

catch ME
    fprintf('\nERROR: %s\n', ME.message);
    if exist('hw', 'var')
        hw.disconnect();
    end
end
