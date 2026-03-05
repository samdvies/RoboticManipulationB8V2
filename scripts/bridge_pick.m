%% bridge_pick.m
% Bridge pick hardware script (MATLAB mirror of Python implementation).
%
% Mirrors:
%   - Bridge no-go decomposition (pillars + deck)
%   - Bridge-safe waypoint planning
%   - Continuous pitch solver with bridge proximity costs (in mode 2)
%   - Smooth segment timing (handled in HardwareInterface mode 2)
%
% Usage:
%   run('scripts/bridge_pick.m')

clc; clear;
scriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(scriptDir, '..', 'src')));

% --- Hardware Configuration ---
PORT      = 'COM4';
BAUD      = 1000000;
VELOCITY  = 20;
MOVE_TIME = 2.5;
Z_FLOOR   = 10;
% Hardware-stable default: execute each planned waypoint with joint interpolation.
% This matches the smoother behavior seen in test_pick_and_place.
MODE_EXEC = 1;        % 1=Joint interpolation, 2=Task-space
USE_DYNAMIC_PITCH = false;  % Only relevant when MODE_EXEC=2

% --- Bridge Geometry ---
BRIDGE_X_MIN = 220.0;
BRIDGE_X_MAX = 230.0;
BRIDGE_Z_MAX = 105.0;
BRIDGE_GAP_Y = 50.0;

bridge_zone = OpenManipulator.BridgeAvoidance.NewZone( ...
    BRIDGE_X_MIN, BRIDGE_X_MAX, -35.0, 35.0, 90.0, BRIDGE_Z_MAX);
bridge_zones = OpenManipulator.BridgeAvoidance.BuildBridgeZones(bridge_zone, BRIDGE_GAP_Y, 10.0, 10.0);

% --- Defaults ---
home_pose = [200, 0, 180, 0];
def_pick  = [225, 0, 30];

fprintf('==========================================\n');
fprintf('  Bridge Pick (Hardware, Python Mirror)\n');
fprintf('==========================================\n');
fprintf('  Bridge: X=[%.1f, %.1f], Z=[%.1f, %.1f]\n', ...
    bridge_zone.x_min, bridge_zone.x_max, bridge_zone.z_min, bridge_zone.z_max);
fprintf('  Exec mode: %d (1=Joint, 2=Task-space)\n', MODE_EXEC);
if MODE_EXEC == 2
    fprintf('  Dynamic pitch optimization: %s\n', string(USE_DYNAMIC_PITCH));
end
fprintf('==========================================\n\n');

% --- User Pick Input ---
fprintf('Enter pick coordinates:\n');
fprintf('  X (default %.0f): ', def_pick(1));
user_in = input('');
if isempty(user_in), pick_x = def_pick(1); else, pick_x = user_in; end

fprintf('  Y (default %.0f): ', def_pick(2));
user_in = input('');
if isempty(user_in), pick_y = def_pick(2); else, pick_y = user_in; end

fprintf('  Z (default %.0f): ', def_pick(3));
user_in = input('');
if isempty(user_in), pick_z = def_pick(3); else, pick_z = user_in; end

pick_pitch = 0.0;
pick_pose = [pick_x, pick_y, pick_z, pick_pitch];
lift_after_pick_pose = [pick_x, pick_y, pick_z + 7.5, pick_pitch];

planner_opts = struct('pitch_tolerance_deg', 5.0, ...
                      'vertical_clearance_mm', 30.0, ...
                      'samples', 20);

wp_entry = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
    home_pose, pick_pose, bridge_zone, bridge_zones, planner_opts);
wp_exit = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
    lift_after_pick_pose, home_pose, bridge_zone, bridge_zones, planner_opts);

fprintf('\nEntry waypoints (%d):\n', size(wp_entry,1));
for i = 1:size(wp_entry,1)
    fprintf('  %d) [%.1f, %.1f, %.1f, %.1f]\n', i, wp_entry(i,1), wp_entry(i,2), wp_entry(i,3), wp_entry(i,4));
end
fprintf('Exit waypoints (%d):\n', size(wp_exit,1));
for i = 1:size(wp_exit,1)
    fprintf('  %d) [%.1f, %.1f, %.1f, %.1f]\n', i, wp_exit(i,1), wp_exit(i,2), wp_exit(i,3), wp_exit(i,4));
end

% Reachability quick check
fprintf('\nChecking key pose reachability...\n');
check_points = {'Home', home_pose; 'Pick', pick_pose; 'LiftAfterPick', lift_after_pick_pose};
all_ok = true;
for i = 1:size(check_points,1)
    name = check_points{i,1}; p = check_points{i,2};
    try
        q = OpenManipulator.IK(p(1), p(2), p(3), p(4), 'elbow_up', false);
        [T, ~] = OpenManipulator.FK(q);
        ee = T(1:3,4)';
        err = norm(ee - p(1:3));
        fprintf('  %-12s -> q=[%6.1f,%6.1f,%6.1f,%6.1f] err=%.1fmm\n', ...
            name, q(1), q(2), q(3), q(4), err);
        if err > 5.0
            fprintf('  %-12s -> REJECTED: IK/FK endpoint error too high (%.1fmm)\n', name, err);
            all_ok = false;
        end
    catch ME
        fprintf('  %-12s -> UNREACHABLE: %s\n', name, ME.message);
        all_ok = false;
    end
end
if ~all_ok
    fprintf('\n*** Some key poses are unreachable. Aborting. ***\n');
    return;
end

input('\nPress ENTER to connect and start...', 's');

try
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    cleanup = onCleanup(@() safeShutdown(hw));
    hw.configure(VELOCITY);
    hw.enableTorque();

    fprintf('\n[PHASE 0] Move HOME and set bridge-safe jaw width...\n');
    hw.moveToPose(home_pose(1), home_pose(2), home_pose(3), home_pose(4), ...
        MOVE_TIME, 1, Z_FLOOR);
    pause(0.5);
    jaw_pct = 50;
    hw.setGripperPosition(jaw_pct);
    pause(0.8);

    fprintf('\n[PHASE 1] Entry route to pick...\n');
    entry_ctx = struct('zones', bridge_zones, ...
                       'final_target_pose', wp_entry(end,:), ...
                       'preplanned_route', true, ...
                       'dynamic_pitch', USE_DYNAMIC_PITCH);
    for i = 1:size(wp_entry,1)
        wp = wp_entry(i,:);
        fprintf('  Entry %d/%d -> [%.1f, %.1f, %.1f, %.1f]\n', ...
            i, size(wp_entry,1), wp(1), wp(2), wp(3), wp(4));
        % Keep the final inward approach as task-space linear to hold Z
        % during slide-in. Other segments can use the default execution mode.
        exec_mode_wp = MODE_EXEC;
        if i == size(wp_entry, 1)
            exec_mode_wp = 2; % Task-space linear final approach
        end
        hw.moveToPose(wp(1), wp(2), wp(3), wp(4), MOVE_TIME, exec_mode_wp, Z_FLOOR, entry_ctx);
        pause(0.3);
    end

    % Final pre-pick lock at exact target pose to remove residual Z offset
    % before user confirms grasp.
    hw.moveToPose(pick_pose(1), pick_pose(2), pick_pose(3), pick_pose(4), ...
        max(0.8, 0.5 * MOVE_TIME), 1, Z_FLOOR, entry_ctx);
    pause(0.2);

    fprintf('\nAt pick position [%.1f, %.1f, %.1f].\n', pick_x, pick_y, pick_z);
    choice = input('Close gripper to pick? (y/n): ', 's');
    if strcmpi(choice, 'y')
        hw.closeGripper();
        pause(1.5);
    end

    fprintf('\n[PHASE 2] Lift 7.5mm...\n');
    lift_ctx = struct('zones', bridge_zones, ...
                      'final_target_pose', lift_after_pick_pose, ...
                      'preplanned_route', true, ...
                      'dynamic_pitch', USE_DYNAMIC_PITCH);
    hw.moveToPose(lift_after_pick_pose(1), lift_after_pick_pose(2), lift_after_pick_pose(3), lift_after_pick_pose(4), ...
        MOVE_TIME, MODE_EXEC, Z_FLOOR, lift_ctx);
    pause(0.3);

    input('Press ENTER to retract to HOME...', 's');

    fprintf('\n[PHASE 3] Exit route to HOME...\n');
    exit_ctx = struct('zones', bridge_zones, ...
                      'final_target_pose', wp_exit(end,:), ...
                      'preplanned_route', true, ...
                      'dynamic_pitch', USE_DYNAMIC_PITCH);
    for i = 1:size(wp_exit,1)
        wp = wp_exit(i,:);
        fprintf('  Exit %d/%d -> [%.1f, %.1f, %.1f, %.1f]\n', ...
            i, size(wp_exit,1), wp(1), wp(2), wp(3), wp(4));
        hw.moveToPose(wp(1), wp(2), wp(3), wp(4), MOVE_TIME, MODE_EXEC, Z_FLOOR, exit_ctx);
        pause(0.3);
    end

    fprintf('\nOpening gripper...\n');
    hw.openGripper();
    pause(0.5);

    hw.disconnect();
    fprintf('\n=== Bridge Pick Complete (Python Mirror) ===\n');

catch ME
    fprintf('\nERROR: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('  in %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
end

%% Helper
function safeShutdown(hw)
    fprintf('\n>>> Cleanup...\n');
    try hw.openGripper(); catch, end
    try hw.disableTorque(); catch, end
    try hw.disconnect(); catch, end
end
