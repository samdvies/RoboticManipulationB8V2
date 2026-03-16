% TEST_TASK2D_CUP2_PICK  Isolated hardware test for Task 2d second cup pickup.
%
% Usage:
%   run('scripts/test_task2d_cup2_pick.m')
%
% This exercises only the holder pickup / return sequence for cup 2 using
% the same tuned geometry as task2d_cups_and_stir.m.

clc; clear;
addpath(genpath('../src'));

% =========================================================================
%  HARDWARE CONFIGURATION
% =========================================================================
PORT        = 'COM4';
BAUD        = 1000000;
VELOCITY    = 50;
MOVE_TIME   = 0.5;
Z_FLOOR     = 15;
MOTION_MODE = 2;  % 1=Joint, 2=Task Linear, 3=Jacobian Hybrid

HOME_POSE = [134, 0, 240, -45];  % [X Y Z Pitch]

% General timing
PAUSE_SHORT = 0.1;
PAUSE_MED   = 0.16;
CUP_GRIP_WIDTH_MM = 36;

% Test options
RETURN_TO_HOLDER = true;   % set false to stop after the vertical pickup
RETURN_HOME_AT_END = true;

% Cup 2 geometry copied from task2d
cup2_x = 200;
cup2_y = 0;
cup2_pick_z = 50;
cup2_base_approach_x = 120;
cup2_mid_approach_x = 160;
cup2_place_z = cup2_pick_z;
cup2_holder_clearance_mm = 50;
cup2_clearance_z = cup2_pick_z + cup2_holder_clearance_mm;
hover2_z = 150;
cup2_hold_pitch_deg = -10;

try
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(PAUSE_SHORT);

    fprintf('[0] Moving to home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('\n=== Cup 2 Pickup Isolation Test ===\n');
    fprintf('Pick pose: [%.1f, %.1f, %.1f] mm\n', cup2_x, cup2_y, cup2_pick_z);
    fprintf('Holder clearance: %.1f mm\n', cup2_holder_clearance_mm);

    pct_cup_grip = (1 - CUP_GRIP_WIDTH_MM / 80) * 100;

    % Approach and pick the cup from the holder.
    hw.openGripper();
    pause(PAUSE_SHORT);
    move_seq(hw, [
        cup2_base_approach_x, cup2_y, hover2_z,            cup2_hold_pitch_deg;
        cup2_mid_approach_x,  cup2_y, cup2_clearance_z,    cup2_hold_pitch_deg;
        cup2_x,               cup2_y, cup2_pick_z,           0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('[1] Gripping cup 2 (%.0fmm)...\n', CUP_GRIP_WIDTH_MM);
    hw.setGripperPosition(pct_cup_grip);
    pause(0.3);

    fprintf('[2] Lifting straight up to clear holder lip...\n');
    move_seq(hw, [
        cup2_x, cup2_y, cup2_clearance_z, 0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    if ~RETURN_TO_HOLDER
        fprintf('[Done] Leaving cup at clearance height for inspection.\n');
        return;
    end

    fprintf('[3] Returning cup 2 to holder...\n');
    move_seq(hw, [
        cup2_x,               cup2_y, cup2_place_z,         0;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    fprintf('[4] Releasing cup 2...\n');
    hw.openGripper();
    pause(0.3);

    fprintf('[5] Retracting in X before lifting away...\n');
    move_seq(hw, [
        cup2_mid_approach_x,  cup2_y, cup2_place_z,         0;
        cup2_mid_approach_x,  cup2_y, cup2_clearance_z,    cup2_hold_pitch_deg;
        cup2_base_approach_x, cup2_y, hover2_z,            cup2_hold_pitch_deg;
    ], MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(PAUSE_MED);

    if RETURN_HOME_AT_END
        fprintf('[6] Returning home...\n');
        hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(PAUSE_MED);
    end

    hw.disconnect();
    fprintf('\n=== Cup 2 pickup test complete ===\n');

catch ME
    fprintf('\nERROR: %s\n', ME.message);
    fprintf('Stack:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s  line %d\n', ME.stack(k).name, ME.stack(k).line);
    end
    if exist('hw', 'var')
        try
            hw.openGripper();
            if RETURN_HOME_AT_END
                hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                              MOVE_TIME, MOTION_MODE, Z_FLOOR);
            end
            hw.disconnect();
        catch
            % best-effort recovery
        end
    end
end

function move_seq(hw, waypoints, move_time, motion_mode, z_floor)
for i = 1:size(waypoints, 1)
    x = waypoints(i, 1);
    y = waypoints(i, 2);
    z = waypoints(i, 3);
    p = waypoints(i, 4);
    hw.moveToPose(x, y, z, p, move_time, motion_mode, z_floor);
end
end
