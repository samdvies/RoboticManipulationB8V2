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
VELOCITY    = 60;       % joint velocity (lower = slower, safer)
MOVE_TIME   = 0.5;      % seconds per waypoint move
Z_FLOOR     = 15;       % safety floor – arm won't go below this Z (mm)
MOTION_MODE = 2;        % 1=Joint, 2=Task Linear, 3=Jacobian Hybrid

HOME_POSE = [134, 0, 240, -45];  % [X Y Z Pitch]

% General timing
PAUSE_SHORT = 0.1;
PAUSE_MED   = 0.16;
CUP_GRIP_WIDTH_MM = 36;
FIRST_CUP_GRIP_WIDTH_MM = 36;
POUR_ANGLE_BOOST_DEG = 20;
FIRST_POUR_ANGLE_BOOST_DEG = 0;
STIR_GRIP_WIDTH_MM = 6;
STIR_PATH_RADIUS_MM = 14.0;
STIR_STREAM_SPEED_MM_S = 155;
STIR_Z_OSCILLATION_MM = 10.0;

try
    % ── Connect ──────────────────────────────────────────────────────────
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(0.16);

    % ── Go Home ─────────────────────────────────────────────────────────
    fprintf('[0] Moving to home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % ====================================================================
    %  DEMO 1: CUP POUR 1  (cup at 75,-175 -> pour into 200,0 -> return)
    % ====================================================================
    fprintf('\n=== Demo 1: Cup Pour 1 ===\n');

    cup1_x = 75;   cup1_y = -175; cup1_z = 30;
    cup1_place_entry_z = 80;
    cup1_place_z = 40;
    cup2_x = 110;  cup2_y = 0;    cup2_z_pour = 130;
    hover_z1 = 150;
    cup1_base_descent_radius = 70;  % descend farther inboard, away from the cup wall
    cup1_final_radius = 43;         % keep the final approach closer to the robot base

    % Approach the first cup from the base side. Descend inboard first,
    % begin pitching up during the descent, then finish with a short
    % diagonal approach into the exact grip pose at the configured pick height.
    cup1_radial = [cup1_x, cup1_y];
    cup1_radial = cup1_radial / norm(cup1_radial);
    cup1_base_entry_xy = [cup1_x, cup1_y] - cup1_base_descent_radius * cup1_radial;
    cup1_final_entry_xy = [cup1_x, cup1_y] - cup1_final_radius * cup1_radial;

    % Approach first cup with gripper open
    hw.openGripper(); pause(PAUSE_SHORT);
    move_seq(hw, [
        cup1_base_entry_xy(1), cup1_base_entry_xy(2), hover_z1, 0;   % go directly to inboard descent line
        cup1_base_entry_xy(1), cup1_base_entry_xy(2), 110,      -90; % start vertical descent
        cup1_base_entry_xy(1), cup1_base_entry_xy(2), 80,       -60; % pitch up during descent
        cup1_final_entry_xy(1), cup1_final_entry_xy(2), 55,     -30; % diagonal low approach
        cup1_x,              cup1_y,              cup1_z,    0;   % exact grip pose
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Grip first cup with a slightly tighter jaw width for better retention.
    fprintf('[D1] Gripping first cup (%.0fmm)...\n', FIRST_CUP_GRIP_WIDTH_MM);
    pct_first_cup_grip = (1 - FIRST_CUP_GRIP_WIDTH_MM/80) * 100;   % assuming 0%%=80mm open, 100%%=0mm
    pct_cup_grip = (1 - CUP_GRIP_WIDTH_MM/80) * 100;   % shared cup 2 grip
    hw.setGripperPosition(pct_first_cup_grip);
    pause(0.33);

    % Carry to second cup and pour, then return first cup
    move_seq(hw, [
        cup1_x, cup1_y, hover_z1, 0;             % lift
        cup2_x, cup2_y, hover_z1, 0;             % above second cup
        cup2_x, cup2_y, hover_z1, -80 - FIRST_POUR_ANGLE_BOOST_DEG;    % start pour
        cup2_x, cup2_y, hover_z1, 0;             % reset upright in place before translating
        cup1_x, cup1_y, cup1_place_entry_z, 0;   % move away only after pitch is neutral
        cup1_x, cup1_y, cup1_place_z,       0;   % straight vertical drop
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Optionally release first cup back on table
    fprintf('[D1] Releasing first cup...\n');
    hw.openGripper();
    pause(0.26);

    % Retract along the same safe inboard side, then rise once clear
    move_seq(hw, [
        cup1_final_entry_xy(1), cup1_final_entry_xy(2), 55,     -30;
        cup1_base_entry_xy(1), cup1_base_entry_xy(2), 80,       -60;
        cup1_base_entry_xy(1), cup1_base_entry_xy(2), hover_z1, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % ====================================================================
    %  DEMO 2: OBJECT PATH / STIRRING
    % ====================================================================
    fprintf('\n=== Demo 2: Stirrer Path ===\n');

    stir_src_x = 150;  stir_src_y = -150;  stir_src_z = 170;
    hover_stir = 240;
    stir_center_x = 190;  stir_center_y = 0;  stir_center_z = 270;

    % Configure gripper for 25 mm stirrer
    fprintf('[D2] Opening gripper for 25mm stirrer...\n');
    hw.openGripper(); pause(PAUSE_SHORT);

    % Approach and pick stirrer
    move_seq(hw, [
        stir_src_x, stir_src_y, hover_stir, 0;
        stir_src_x, stir_src_y, stir_src_z, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('[D2] Gripping stirrer (%.0fmm)...\n', STIR_GRIP_WIDTH_MM);
    pct_stir_grip = (1 - STIR_GRIP_WIDTH_MM/80) * 100;
    hw.setGripperPosition(pct_stir_grip);
    pause(0.26);

    % Move up and across to stirring centre
    move_seq(hw, [
        stir_src_x,       stir_src_y,   hover_stir,     0;
        stir_src_x,              0,     stir_center_z,  0;
        stir_center_x, stir_center_y,   stir_center_z,  0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Stirring loop: approximate a circle with many short segments so the
    % motion feels continuous rather than corner-to-corner.
    stir_path_radius = STIR_PATH_RADIUS_MM;
    stir_path_points = 16;
    stir_path_angles = linspace(0, 2*pi, stir_path_points + 1);
    stir_path_angles(end) = [];
    path_points = [
        stir_center_x + stir_path_radius * cos(stir_path_angles(:)), ...
        stir_center_y + stir_path_radius * sin(stir_path_angles(:)), ...
        180 + STIR_Z_OSCILLATION_MM * sin(stir_path_angles(:))
    ];

    fprintf('[D2] Running circular stirring path...\n');
    stir_entry_pose = [path_points(1, 1), path_points(1, 2), path_points(1, 3), 0];
    stir_waypoints = [path_points, zeros(size(path_points, 1), 1)];
    stir_waypoints_reverse = flipud(stir_waypoints);
    stir_stream_path = [
        stir_entry_pose;
        repmat(stir_waypoints, 4, 1);
        stir_waypoints(1, :);
        repmat(stir_waypoints_reverse, 4, 1);
        stir_waypoints_reverse(1, :)
    ];
    stream_pose_path(hw, stir_stream_path, STIR_STREAM_SPEED_MM_S, 0.02, Z_FLOOR);

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
    pause(0.26);

    move_seq(hw, [
        stir_src_x, stir_src_y, hover_stir, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % ====================================================================
    %  DEMO 3: CUP POUR 2 (to "mouth" arc)
    % ====================================================================
    fprintf('\n=== Demo 3: Cup Pour 2 (to mouth) ===\n');

    cup2_x = 200;  cup2_y = 0;   cup2_pick_z = 50;
    hover2_z = 150;
    cup2_base_approach_x = 120;
    cup2_mid_approach_x = 160;
    cup2_place_z = cup2_pick_z;
    cup2_holder_clearance_mm = 50;
    cup2_clearance_z = cup2_pick_z + cup2_holder_clearance_mm;
    cup2_hold_pitch_deg = -10;

    % "Mouth" arc poses (X, Y, Z, Pitch), shifted 50 mm inward along radius
    mouth_start_xy = [150, 150];
    mouth_mid_xy   = [175, 175];
    mouth_end_xy   = [200, 200];
    mouth_radial_offset = 50;

    mouth_start_xy = mouth_start_xy - mouth_radial_offset * (mouth_start_xy / norm(mouth_start_xy));
    mouth_mid_xy   = mouth_mid_xy   - mouth_radial_offset * (mouth_mid_xy   / norm(mouth_mid_xy));
    mouth_end_xy   = mouth_end_xy   - mouth_radial_offset * (mouth_end_xy   / norm(mouth_end_xy));

    mouth_start = [mouth_start_xy(1), mouth_start_xy(2), 100,   0];
    mouth_mid   = [mouth_mid_xy(1),   mouth_mid_xy(2),   150, -60 - POUR_ANGLE_BOOST_DEG];
    mouth_end   = [mouth_end_xy(1),   mouth_end_xy(2),   150, -70 - POUR_ANGLE_BOOST_DEG];

    % Approach and pick second cup
    hw.openGripper(); pause(PAUSE_SHORT);
    move_seq(hw, [
        cup2_base_approach_x, cup2_y, hover2_z,  cup2_hold_pitch_deg;
        cup2_mid_approach_x,  cup2_y, cup2_clearance_z, cup2_hold_pitch_deg;
        cup2_x,               cup2_y, cup2_pick_z,       0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('[D3] Gripping second cup (%.0fmm)...\n', CUP_GRIP_WIDTH_MM);
    hw.setGripperPosition(pct_cup_grip);
    pause(0.26);

    % Move from pickup to mouth_start
    move_seq(hw, [
        cup2_x,               cup2_y, cup2_clearance_z,  0;
        cup2_mid_approach_x,  cup2_y, cup2_clearance_z, cup2_hold_pitch_deg;
        cup2_base_approach_x, cup2_y, hover2_z,         cup2_hold_pitch_deg;
        mouth_start(1), mouth_start(2), mouth_start(3), mouth_start(4);
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Three sip cycles along the arc and back
    fprintf('[D3] Performing mouth pour cycles...\n');
    for cycle = 1:3
        move_seq(hw, [
            mouth_end(1),  mouth_end(2),  mouth_end(3),  mouth_end(4);
            mouth_start(1), mouth_start(2), mouth_start(3), mouth_start(4);
        ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    end

    % Return cup 2 to its original pickup position
    fprintf('[D3] Returning second cup...\n');
    move_seq(hw, [
        cup2_base_approach_x, cup2_y, hover2_z,  cup2_hold_pitch_deg;
        cup2_mid_approach_x,  cup2_y, cup2_clearance_z, cup2_hold_pitch_deg;
        cup2_x,               cup2_y, cup2_clearance_z,  0;
        cup2_x,               cup2_y, cup2_place_z, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    % Release second cup
    hw.openGripper();
    pause(0.26);

    % Retract in X first so the released cup can settle in the holder
    % before the arm lifts away vertically.
    move_seq(hw, [
        cup2_mid_approach_x,  cup2_y, cup2_place_z,      0;
        cup2_mid_approach_x,  cup2_y, cup2_clearance_z, cup2_hold_pitch_deg;
        cup2_base_approach_x, cup2_y, hover2_z,  cup2_hold_pitch_deg;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

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

function stream_pose_path(hw, waypoints, speed_mm_s, dt, z_floor)
if isempty(waypoints)
    return;
end

q_current = hw.readAngles();
[T_current, ~] = OpenManipulator.FK(q_current);
prev_pose = [T_current(1:3, 4)', -(q_current(2) + q_current(3) + q_current(4))];

for i = 1:size(waypoints, 1)
    target_pose = waypoints(i, :);
    dist_lin = norm(target_pose(1:3) - prev_pose(1:3));
    dist_rot = abs(target_pose(4) - prev_pose(4));
    duration = max([dist_lin / max(speed_mm_s, 1e-6), dist_rot / 90.0, dt]);
    num_steps = max(1, ceil(duration / dt));

    for step = 1:num_steps
        s = step / num_steps;
        s_smooth = s * s * (3.0 - 2.0 * s);
        pose = (1 - s_smooth) * prev_pose + s_smooth * target_pose;

        if pose(3) < z_floor
            error('Motion Safety Violation: Commanded Z (%.1f mm) < %.1f mm. Aborting.', pose(3), z_floor);
        end

        q_interp = OpenManipulator.IK(pose(1), pose(2), pose(3), pose(4));
        [q_interp, ~] = OpenManipulator.JointLimits.Clamp(q_interp);
        encoders = zeros(1, 4);
        for joint_idx = 1:4
            encoders(joint_idx) = OpenManipulator.HardwareInterface.deg2enc(q_interp(joint_idx));
        end
        hw.syncWritePositions(encoders);
        pause(dt);
    end

    prev_pose = target_pose;
end

q_final = OpenManipulator.IK(prev_pose(1), prev_pose(2), prev_pose(3), prev_pose(4));
[q_final, ~] = OpenManipulator.JointLimits.Clamp(q_final);
encoders = zeros(1, 4);
for joint_idx = 1:4
    encoders(joint_idx) = OpenManipulator.HardwareInterface.deg2enc(q_final(joint_idx));
end
for tail = 1:5
    hw.syncWritePositions(encoders);
    pause(dt);
end
hw.waitForMotion();
end

