% TASK2D_CUPS_AND_STIR  Cup pour and stirring demo using hardware
%
% Usage:
%   run('scripts/task2d_cups_and_stir.m')
%
% This script mirrors the three scripted demos in the Python visualisation:
%   1. Cup Pour 1: pick first cup at (75, -175, ~60) and pour into cup at
%      (200, 0), then return the first cup to its original pose.
%   2. Object Path / Stirring: pick a 25 mm stirrer at (150, -150, 170),
%      move it to (200, 0, 270) and run a circular stirring path, then
%      return the stirrer to its original pose.
%   3. Cup Pour 2: pick the second cup at (200, 0, 60), move it towards a
%      "mouth" region in front of the robot, and perform three natural-looking
%      pour motions along an arc.
%
% Coordinates are in the same user frame as the simulator, in mm and degrees.

clc; clear;
addpath(genpath('../src'));

% =========================================================================
%  HARDWARE CONFIGURATION
% =========================================================================
PORT        = 'COM4';
BAUD        = 1000000;
VELOCITY    = 20;       % joint velocity (lower = slower, safer)
MOVE_TIME   = 1.5;      % seconds per waypoint move
Z_FLOOR     = 15;       % safety floor – arm won't go below this Z (mm)
MOTION_MODE = 2;        % 1=Joint, 2=Task Linear, 3=Jacobian Hybrid

HOME_POSE = [134, 0, 240, -45];  % [X Y Z Pitch]

% General timing
PAUSE_SHORT = 0.3;
PAUSE_MED   = 0.5;

try
    % ── Connect ──────────────────────────────────────────────────────────
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(0.5);

    % ── Go Home ─────────────────────────────────────────────────────────
    fprintf('[0] Moving to home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % ====================================================================
    %  DEMO 1: CUP POUR 1  (cup at 75,-175 -> pour into 200,0 -> return)
    % ====================================================================
    fprintf('\n=== Demo 1: Cup Pour 1 ===\n');

    cup1_x = 75;   cup1_y = -175; cup1_z = 60;
    cup2_x = 200;  cup2_y = 0;    cup2_z_pour = 130;
    hover_z1 = 150;

    % Approach first cup with gripper open
    hw.openGripper(); pause(PAUSE_SHORT);
    move_seq(hw, [
        cup1_x, cup1_y, hover_z1, 0;      % above cup
        cup1_x, cup1_y, 100,      0;      % just above rim
        cup1_x, cup1_y, cup1_z,   0;      % at cup height
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Grip first cup (approx 60 mm wide)
    fprintf('[D1] Gripping first cup...\n');
    hw.closeGripper();
    pause(1.0);

    % Carry to second cup and pour, then return first cup
    move_seq(hw, [
        cup1_x, cup1_y, hover_z1, 0;             % lift
        cup2_x, cup2_y, hover_z1, 0;             % above second cup
        cup2_x, cup2_y, cup2_z_pour, 0;          % lower a bit
        cup2_x, cup2_y, cup2_z_pour, -60;        % start pour
        cup2_x, cup2_y, cup2_z_pour, -90;        % full pour
        cup2_x, cup2_y, hover_z1, 0;             % upright and lift
        cup1_x, cup1_y, hover_z1, 0;             % back above first cup
        cup1_x, cup1_y, cup1_z,   0;             % back to original height
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Optionally release first cup back on table
    fprintf('[D1] Releasing first cup...\n');
    hw.openGripper();
    pause(0.8);

    % Lift back up before next demo
    move_seq(hw, [
        cup1_x, cup1_y, hover_z1, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % ====================================================================
    %  DEMO 2: OBJECT PATH / STIRRING
    % ====================================================================
    fprintf('\n=== Demo 2: Stirrer Path ===\n');

    stir_src_x = 150;  stir_src_y = -150;  stir_src_z = 170;
    hover_stir = 210;
    stir_center_x = 200;  stir_center_y = 0;  stir_center_z = 270;

    % Configure gripper for 25 mm stirrer
    fprintf('[D2] Opening gripper for 25mm stirrer...\n');
    hw.openGripper(); pause(PAUSE_SHORT);

    % Approach and pick stirrer
    move_seq(hw, [
        stir_src_x, stir_src_y, hover_stir, 0;
        stir_src_x, stir_src_y, stir_src_z, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('[D2] Gripping stirrer...\n');
    hw.closeGripper();
    pause(0.8);

    % Move up and across to stirring centre
    move_seq(hw, [
        stir_src_x,       stir_src_y,   hover_stir,     0;
        stir_src_x,              0,     stir_center_z,  0;
        stir_center_x, stir_center_y,   stir_center_z,  0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Stirring loop: 4 laps around the 8-point path at Z=180, pitch=0
    path_points = [
        207.500,   3.107, 180;
        203.107,   7.500, 180;
        196.893,   7.500, 180;
        192.500,   3.107, 180;
        192.500,  -3.107, 180;
        196.893,  -7.500, 180;
        203.107,  -7.500, 180;
        207.500,  -3.107, 180;
    ];

    fprintf('[D2] Running stirring path...\n');
    for lap = 1:4
        for k = 1:size(path_points,1)
            px = path_points(k,1);
            py = path_points(k,2);
            pz = path_points(k,3);
            hw.moveToPose(px, py, pz, 0, ...
                          MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(PAUSE_SHORT);
        end
    end

    % Return stirrer to original pose
    move_seq(hw, [
        stir_center_x, stir_center_y, stir_center_z, 0;
        stir_src_x,              0,   stir_center_z, 0;
        stir_src_x,       stir_src_y, hover_stir,    0;
        stir_src_x,       stir_src_y, stir_src_z,    0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('[D2] Releasing stirrer...\n');
    hw.openGripper();
    pause(0.8);

    move_seq(hw, [
        stir_src_x, stir_src_y, hover_stir, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % ====================================================================
    %  DEMO 3: CUP POUR 2 (to "mouth" arc)
    % ====================================================================
    fprintf('\n=== Demo 3: Cup Pour 2 (to mouth) ===\n');

    cup2_x = 200;  cup2_y = 0;   cup2_z = 60;
    hover2_z = 150;

    % "Mouth" arc poses (X, Y, Z, Pitch)
    mouth_start = [150, 150, 100,   0];
    mouth_mid   = [175, 175, 125, -45];
    mouth_end   = [200, 200, 150, -90];

    % Approach and pick second cup
    hw.openGripper(); pause(PAUSE_SHORT);
    move_seq(hw, [
        cup2_x, cup2_y, hover2_z, 0;
        cup2_x, cup2_y, 100,      0;
        cup2_x, cup2_y, cup2_z,   0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('[D3] Gripping second cup...\n');
    hw.closeGripper();
    pause(0.8);

    % Move from pickup to mouth_start
    move_seq(hw, [
        cup2_x, cup2_y, hover2_z,          0;
        mouth_start(1), mouth_start(2), mouth_start(3), mouth_start(4);
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Three sip cycles along the arc and back
    fprintf('[D3] Performing mouth pour cycles...\n');
    for cycle = 1:3
        move_seq(hw, [
            mouth_mid(1),  mouth_mid(2),  mouth_mid(3),  mouth_mid(4);
            mouth_end(1),  mouth_end(2),  mouth_end(3),  mouth_end(4);
            mouth_mid(1),  mouth_mid(2),  mouth_mid(3),  mouth_mid(4);
            mouth_start(1), mouth_start(2), mouth_start(3), mouth_start(4);
        ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    end

    % Finish by returning to home
    fprintf('\n[Done] Returning home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    hw.disconnect();
    fprintf('\n=== Task 2d complete ===\n');

catch ME
    fprintf('\nERROR: %s\n', ME.message);
    fprintf('Stack:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s  line %d\n', ME.stack(k).name, ME.stack(k).line);
    end
    if exist('hw', 'var')
        try
            hw.openGripper();
            hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                          MOVE_TIME, MOTION_MODE, Z_FLOOR);
            hw.disconnect();
        catch
            % best-effort recovery
        end
    end
end

% ========================================================================
%  Helper: move through a sequence of [x y z pitch] rows
% ========================================================================
function move_seq(hw, waypoints, move_time, motion_mode, z_floor)
for i = 1:size(waypoints, 1)
    x = waypoints(i, 1);
    y = waypoints(i, 2);
    z = waypoints(i, 3);
    p = waypoints(i, 4);
    hw.moveToPose(x, y, z, p, move_time, motion_mode, z_floor);
end
end

