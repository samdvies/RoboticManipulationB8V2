%% test_bridge_pick.m
% Pick an object from under a bridge / overhead obstacle.
%
% Strategy:
%   The arm must NOT go above the bridge height near the pick position.
%   Instead of the usual "lift up -> move across -> lower down" approach,
%   the arm approaches horizontally:
%
%   1. Move to a safe HOME/staging position (clear of bridge)
%   2. Lower to APPROACH height while still clear of the bridge
%   3. SLIDE IN horizontally to the pick position (task-space linear)
%   4. Grip the object
%   5. RETRACT horizontally back to the approach position
%   6. Lift to a safe height and place
%
% Geometry note:
%   With pitch=0 (horizontal gripper), the 126mm tool link extends forward.
%   The minimum reachable X is ~140mm. Plan approach positions accordingly.
%
% Usage:
%   run('scripts/test_bridge_pick.m')

clc; clear;
addpath(genpath('../src'));

% --- Configuration ---
PORT     = 'COM4';
BAUD     = 1000000;
VELOCITY = 20;

MOVE_TIME    = 2.0;     % seconds per move segment
SLIDE_TIME   = 3.0;     % seconds for the horizontal slide (slower for precision)
MOTION_MODE  = 2;       % 2 = Task-space linear (ensures straight line)
Z_FLOOR      = 10;      % mm, EE floor safety

% --- Define Positions ---
% Modify these to match your setup:

% Home: safe position, comfortably within reach
home_pose = [200, 0, 180, 0];  % pitch=0 for easy transition

% Pick parameters
pick_x     = 200;       % X of the object (mm)
pick_y     = 0;         % Y of the object (mm)
pick_z     = 50;        % Z of the object (mm) — under the bridge
pick_pitch = 0;         % Horizontal approach

% Bridge parameters
bridge_z = pick_z + 50; % Bridge is at this Z (mm). Arm stays BELOW near pick.

% Approach position: FURTHER OUT (larger X) so we slide IN toward the pick.
% This keeps the arm on the far side, clear of any bridge between robot & object.
approach_offset = 60;   % mm further out from pick
approach_x = pick_x + approach_offset;   % Slide IN from here

% Place position (after pick)
place_pose = [200, 100, 120, 0];

fprintf('==========================================\n');
fprintf('  Bridge Pick Test\n');
fprintf('==========================================\n');
fprintf('  Pick:      [%.0f, %.0f, %.0f] pitch=%.0f\n', pick_x, pick_y, pick_z, pick_pitch);
fprintf('  Bridge Z:  %.0f mm (arm stays below near pick)\n', bridge_z);
fprintf('  Approach:  slide in from X=%.0f -> X=%.0f\n', approach_x, pick_x);
fprintf('==========================================\n\n');

% Quick reachability check
fprintf('Checking positions...\n');
positions = {
    'Home',     home_pose(1), home_pose(2), home_pose(3), home_pose(4);
    'Approach', approach_x,   pick_y,       pick_z,       pick_pitch;
    'Pick',     pick_x,       pick_y,       pick_z,       pick_pitch;
};
for i = 1:size(positions, 1)
    name = positions{i, 1};
    px = positions{i, 2}; py = positions{i, 3};
    pz = positions{i, 4}; pp = positions{i, 5};
    try
        q = OpenManipulator.IK(px, py, pz, pp);
        [T, ~] = OpenManipulator.FK(q);
        ee = T(1:3, 4)';
        err = norm(ee - [px, py, pz]);
        fprintf('  %-10s [%3.0f,%3.0f,%3.0f] p=%3.0f -> q=[%5.1f,%5.1f,%5.1f,%5.1f] err=%.1fmm OK\n', ...
            name, px, py, pz, pp, q(1), q(2), q(3), q(4), err);
    catch ME
        fprintf('  %-10s [%3.0f,%3.0f,%3.0f] p=%3.0f -> UNREACHABLE: %s\n', ...
            name, px, py, pz, pp, ME.message);
    end
end
fprintf('\n');

try
    % --- Connect & Configure ---
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    cleanup = onCleanup(@() safeShutdown(hw));
    hw.configure(VELOCITY);
    hw.enableTorque();

    % --- Phase 0: Go Home ---
    fprintf('\n[PHASE 0] Moving to HOME position...\n');
    hw.moveToPose(home_pose(1), home_pose(2), home_pose(3), home_pose(4), ...
        MOVE_TIME, 1, Z_FLOOR);
    hw.openGripper();
    pause(1);

    % --- Phase 1: Move to Approach Position ---
    % Lower to pick height at approach_x (further out, clear of bridge)
    fprintf('[PHASE 1] Moving to APPROACH position [%.0f, %.0f, %.0f]...\n', ...
        approach_x, pick_y, pick_z);

    % Step 1a: Move to approach X/Y at safe height first
    mid_z = max(pick_z + 60, bridge_z + 30);
    hw.moveToPose(approach_x, pick_y, mid_z, pick_pitch, ...
        MOVE_TIME, 1, Z_FLOOR);
    pause(0.5);

    % Step 1b: Lower to approach height
    hw.moveToPose(approach_x, pick_y, pick_z, pick_pitch, ...
        MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    fprintf('  At approach position. Ready to slide in.\n');
    input('  Press ENTER to slide in to pick position...', 's');

    % --- Phase 2: Slide In Horizontally ---
    % Task-space linear mode ensures straight-line path.
    % Z stays constant, pitch stays at 0 — pure horizontal slide.
    fprintf('[PHASE 2] SLIDING IN to pick position [%.0f, %.0f, %.0f]...\n', ...
        pick_x, pick_y, pick_z);

    q_pick = OpenManipulator.IK(pick_x, pick_y, pick_z, pick_pitch);
    hw.executePoseMove(q_pick, [pick_x, pick_y, pick_z], pick_pitch, ...
        SLIDE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    fprintf('  At pick position.\n');
    choice = input('  Pick object? (y/n): ', 's');

    if strcmpi(choice, 'y')
        % --- Phase 3: Grip ---
        fprintf('[PHASE 3] Gripping...\n');
        hw.closeGripper();
        pause(2);

        % --- Phase 4: Retract Horizontally ---
        fprintf('[PHASE 4] RETRACTING horizontally...\n');
        q_approach = OpenManipulator.IK(approach_x, pick_y, pick_z, pick_pitch);
        hw.executePoseMove(q_approach, [approach_x, pick_y, pick_z], pick_pitch, ...
            SLIDE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.5);

        % --- Phase 5: Lift & Move to Place ---
        fprintf('[PHASE 5] Lifting and moving to PLACE position...\n');
        hw.moveToPose(approach_x, pick_y, mid_z, pick_pitch, ...
            MOVE_TIME, 1, Z_FLOOR);
        pause(0.5);

        hw.moveToPose(place_pose(1), place_pose(2), place_pose(3), place_pose(4), ...
            MOVE_TIME, 1, Z_FLOOR);
        pause(0.5);

        fprintf('  At place position.\n');
        input('  Press ENTER to release...', 's');

        % --- Phase 6: Release ---
        fprintf('[PHASE 6] Releasing...\n');
        hw.openGripper();
        pause(1);

        hw.moveToPose(place_pose(1), place_pose(2), place_pose(3) + 60, place_pose(4), ...
            MOVE_TIME, 1, Z_FLOOR);
    else
        fprintf('  Skipping pick.\n');
    end

    % --- Return Home ---
    fprintf('\n[DONE] Returning HOME...\n');
    hw.moveToPose(home_pose(1), home_pose(2), home_pose(3), home_pose(4), ...
        MOVE_TIME, 1, Z_FLOOR);
    hw.openGripper();
    hw.disconnect();
    fprintf('=== Bridge Pick Complete ===\n');

catch ME
    fprintf('\nERROR: %s\n', ME.message);
    fprintf('  in %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
end

%% Helper
function safeShutdown(hw)
    fprintf('\n>>> Cleanup...\n');
    try hw.openGripper(); catch, end
    try hw.disableTorque(); catch, end
    try hw.disconnect(); catch, end
end
