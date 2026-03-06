%% task2c_tool_through_gates.m
% Task 2.c — Tool Through Gates (Constant-Z XY Motion)
%
% Drives the OpenManipulator-X to pick a tool and move it through a series
% of gates at constant Z height, then places it at the drop zone.
%
% Usage (from MATLAB, in project root):
%   run('scripts/task2c_tool_through_gates.m')
%
% Set DRY_RUN = true to validate IK/FK for all waypoints without hardware.
%
% --- Jitter (Mode 2 / Mode 3) ---
% If motion looks jittery: (1) Raise VELOCITY (e.g. 50 or 80) so servos track
% the stream of targets; (2) In HardwareInterface.executePoseMove set
% debug_jitter = true to log commanded vs actual joint angles and err_mm
% (persistent large err_mm = lag; oscillating err_mm = overshoot).

clear; clc;

%% ======================== CONFIGURATION ========================

% --- Hardware ---
COM_PORT  = 'COM4';
BAUDRATE  = 1000000;
VELOCITY  = 50;           % Dynamixel profile velocity. Higher = servos track Mode 2/3 targets better (reduces jitter). Try 40-80; 20 was too low.
DRY_RUN   = false;        % true = IK/FK validation only, no hardware

% --- Motion ---
% MOTION_MODE for fly legs (5,9) and gate_traverse (7): straight line in Cartesian
% space so tool stays at SAFE_Z when flying, and goes straight through gates.
% Other moves use Mode 1. 2=Task-space linear, 3=Jacobian.
MOTION_MODE   = 3;
MOVE_TIME     = 4.0;      % Max time per segment (s). Actual time scales with distance (~70 mm/s nominal) for straight-line.
Z_FLOOR       = 20.0;     % EE Z-floor safety limit (mm)

% --- Task Coordinates (mm) ---
TOOL_PICK_XY  = [-50, -200];     % Tool pickup location (XY)
GRASP_Z       = 75;              % Z to grasp (matches previous approach height)
SAFE_Z        = 105;             % Approach above tool (was 75; +30 for clearance)
FLY_Z         = 110;              % Z when flying between locations (higher = clearer)
GATE_Z        = 70;              % (unused) gate Z now varies by segment
BRIDGE_Z_LOW  = 105;             % Pickup height; hold for first and last bridge segments
BRIDGE_Z_HIGH = 130;             % +25 mm for central two bridges only

% Gate waypoints — bridge 1, bridge 2 to (225,0), then straight to (175,0), then to drop approach
GATE_WAYPOINTS = [
    100, -175;    % Waypoint 1
    100,  -75;    % Waypoint 2 (end bridge 1)
    225,    0;    % Waypoint 3 (end bridge 2)
    175,    0;    % Waypoint 4 — straight from (225,0)
    175,  100;    % Waypoint 5
];

DROP_XY  = [50, 100];     % Drop-off location (XY)
DROP_Z   = 50;            % Z for releasing tool

% --- End-Effector Orientation ---
PITCH        = -80;            % Tilted tool (to reach far bridge); reduced from -85 to avoid bridge collision
PITCH_BRIDGE2 = PITCH + 1;     % -79° on second bridge only (1° less tilt)
PITCH_BRIDGE3 = PITCH - 1;     % -81° on third bridge (225,0)->(175,0), 1° closer to -90

%% ======================== ADD SOURCE PATH ========================

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

%% ======================== BUILD WAYPOINT LIST ========================
% Straight-line only through the gates (bridges); between gates we lift and
% move normally (point-to-point). Fewer waypoints, faster, fewer segment jumps.
%
% label_id: 1=approach, 2=lower_grasp, 3=grip, 4=lift, 5=fly_to_gate1,
%           6=lower_gate, 7=gate_traverse (straight-line only), 8=lift_after,
%           9=fly_to_drop, 10=lower_drop, 11=release, 12=lift_final

LABELS = {'approach_above_tool', 'lower_to_grasp', 'close_gripper', ...
          'lift_to_safe_z', 'fly_to_gate1', 'lower_to_gate_z', ...
          'gate_traverse', 'lift_after_gates', 'fly_to_drop', ...
          'lower_to_drop', 'open_gripper', 'lift_and_home', ...
          'lift_for_central_bridges', 'lower_for_final_bridges'};

waypoints = [];
phase_labels = [];

% 1. Approach above tool at SAFE_Z
waypoints(end+1, :) = [TOOL_PICK_XY, SAFE_Z, PITCH];
phase_labels(end+1) = 1;

% 2. Lower to grasp
waypoints(end+1, :) = [TOOL_PICK_XY, GRASP_Z, PITCH];
phase_labels(end+1) = 2;

% 3. Close gripper (marker waypoint — same position)
waypoints(end+1, :) = [TOOL_PICK_XY, GRASP_Z, PITCH];
phase_labels(end+1) = 3;

% 4. Lift to SAFE_Z
waypoints(end+1, :) = [TOOL_PICK_XY, SAFE_Z, PITCH];
phase_labels(end+1) = 4;

% 5. Fly to first gate at pickup height (no Z drop)
waypoints(end+1, :) = [GATE_WAYPOINTS(1,:), SAFE_Z, PITCH];
phase_labels(end+1) = 5;

% 6–7. Traverse gates: segment 1 single move; segment 2 (100,-75)->(225,0) kept with intermediates for pitch tuning
% Segment 1: wp1 -> wp2 at BRIDGE_Z_LOW (one waypoint = one smooth straight line)
waypoints(end+1, :) = [GATE_WAYPOINTS(2,:), BRIDGE_Z_LOW, PITCH];
phase_labels(end+1) = 7;
% Lift for central two bridges
waypoints(end+1, :) = [GATE_WAYPOINTS(2,:), BRIDGE_Z_HIGH, PITCH];
phase_labels(end+1) = 13;
% Segment 2: wp2 -> wp3 at BRIDGE_Z_HIGH — UNCHANGED (appendManhattanXY + PITCH_BRIDGE2)
[waypoints, phase_labels] = appendManhattanXY( ...
    waypoints, phase_labels, GATE_WAYPOINTS(2,:), GATE_WAYPOINTS(3,:), BRIDGE_Z_HIGH, PITCH_BRIDGE2, 7);
% Segment 3: wp3 -> wp4 straight (225,0) -> (175,0) at BRIDGE_Z_HIGH (one waypoint, third bridge pitch)
waypoints(end+1, :) = [GATE_WAYPOINTS(4,:), BRIDGE_Z_HIGH, PITCH_BRIDGE3];
phase_labels(end+1) = 7;
% Segment 4: wp4 -> wp5 at BRIDGE_Z_HIGH (one waypoint)
waypoints(end+1, :) = [GATE_WAYPOINTS(5,:), BRIDGE_Z_HIGH, PITCH];
phase_labels(end+1) = 7;

% 8. Lift after gates
waypoints(end+1, :) = [GATE_WAYPOINTS(end,:), FLY_Z, PITCH];
phase_labels(end+1) = 8;

% 9. Fly to above drop zone
waypoints(end+1, :) = [DROP_XY, FLY_Z, PITCH];
phase_labels(end+1) = 9;

% 10. Lower to drop Z
waypoints(end+1, :) = [DROP_XY, DROP_Z, PITCH];
phase_labels(end+1) = 10;

% 11. Release (marker waypoint — same position)
waypoints(end+1, :) = [DROP_XY, DROP_Z, PITCH];
phase_labels(end+1) = 11;

% 12. Lift and home
waypoints(end+1, :) = [DROP_XY, FLY_Z, PITCH];
phase_labels(end+1) = 12;

num_waypoints = size(waypoints, 1);

%% ======================== DRY-RUN IK/FK VALIDATION ========================

fprintf('========================================================\n');
fprintf('  Task 2.c — Tool Through Gates\n');
fprintf('========================================================\n\n');

fprintf('--- IK/FK Dry-Run Validation (%d waypoints) ---\n\n', num_waypoints);

max_fk_err = 0;
all_valid  = true;

fprintf('  %-4s  %-22s  %-28s  %-28s  %s\n', ...
    '#', 'Phase', 'Target [x, y, z]', 'IK Solution [q1..q4]°', 'FK Error');
fprintf('  %s\n', repmat('-', 1, 110));

for i = 1:num_waypoints
    x = waypoints(i, 1);
    y = waypoints(i, 2);
    z = waypoints(i, 3);
    p = waypoints(i, 4);
    label = LABELS{phase_labels(i)};

    % Compute IK
    q = OpenManipulator.IK(x, y, z, p);

    % Validate joints
    [is_valid, violations] = OpenManipulator.JointLimits.Validate(q);

    % FK verification
    [T_ee, ~] = OpenManipulator.FK(q);
    fk_pos = T_ee(1:3, 4)';
    fk_err = norm(fk_pos - [x, y, z]);
    max_fk_err = max(max_fk_err, fk_err);

    % Status indicator
    if ~is_valid || fk_err > 5.0
        status = '✗';
        all_valid = false;
    else
        status = '✓';
    end

    fprintf('  %s%-3d  %-22s  [%7.1f, %7.1f, %7.1f]  [%6.1f, %6.1f, %6.1f, %6.1f]  %.2f mm\n', ...
        status, i, label, x, y, z, q(1), q(2), q(3), q(4), fk_err);

    if ~is_valid
        for v = 1:length(violations)
            viol = violations{v};
            fprintf('      ⚠ Joint %d (%s): %.1f° exceeds [%.0f°, %.0f°]\n', ...
                viol.joint, viol.name, viol.angle, viol.min, viol.max);
        end
    end
end

fprintf('\n  Max FK Error: %.3f mm\n', max_fk_err);

if all_valid
    fprintf('  ✓ All waypoints valid.\n\n');
else
    fprintf('  ✗ Some waypoints have issues — review above.\n\n');
end

if DRY_RUN
    fprintf('=== DRY_RUN mode — skipping hardware. ===\n');
    return;
end

%% ======================== HARDWARE EXECUTION ========================

fprintf('--- Connecting to Hardware ---\n\n');

hw = OpenManipulator.HardwareInterface(COM_PORT, BAUDRATE);
cleanup = onCleanup(@() safeShutdown(hw));

hw.configure(VELOCITY);
hw.enableTorque();

% Move to HOME first
fprintf('\nMoving to HOME...\n');
hw.moveHome();
pause(1);

% Open gripper ready for pickup
hw.openGripper();
pause(1);

fprintf('\n========================================================\n');
fprintf('  Executing Task 2.c Sequence (%d waypoints)\n', num_waypoints);
fprintf('========================================================\n\n');

for i = 1:num_waypoints
    x = waypoints(i, 1);
    y = waypoints(i, 2);
    z = waypoints(i, 3);
    p = waypoints(i, 4);
    label = LABELS{phase_labels(i)};
    lid   = phase_labels(i);

    fprintf('[%2d/%2d] %-22s  Target: [%.1f, %.1f, %.1f] pitch=%.0f°\n', ...
        i, num_waypoints, label, x, y, z, p);

    % --- Gripper actions (marker waypoints) ---
    if lid == 3   % close_gripper
        fprintf('        Closing gripper...\n');
        hw.closeGripper();
        pause(2);
        fprintf('        Gripper: %.0f%%\n', hw.readGripperPosition());
        continue;
    end

    if lid == 11  % open_gripper
        fprintf('        Opening gripper...\n');
        hw.openGripper();
        pause(1);
        fprintf('        Gripper: %.0f%%\n', hw.readGripperPosition());
        continue;
    end

    % --- Arm movement ---
    % Cartesian (Mode 2/3) for fly legs (5,9) and gate_traverse (7). Rest use Mode 1.
    % Fly sections (5,9) use higher speed limits so flying is obviously faster than bridge.
    if ismember(phase_labels(i), [5, 7, 9])
        mode = MOTION_MODE;
    else
        mode = 1;
    end
    is_fly = ismember(phase_labels(i), [5, 9]);  % Fly-to-gate and fly-to-drop: no speed cap

    try
        % Compute IK for logging
        q_target = OpenManipulator.IK(x, y, z, p);
        fprintf('        IK: [%.1f, %.1f, %.1f, %.1f]°\n', q_target);

        % Move
        hw.moveToPose(x, y, z, p, MOVE_TIME, mode, Z_FLOOR, is_fly);

        % Read actual position
        q_actual = hw.readAngles();
        [T_actual, ~] = OpenManipulator.FK(q_actual);
        actual_pos = T_actual(1:3, 4)';
        hw_err = norm(actual_pos - [x, y, z]);

        fprintf('        Actual: [%.1f, %.1f, %.1f] mm  (error: %.1f mm)\n', ...
            actual_pos, hw_err);

        if hw_err > 10
            fprintf('        ⚠ WARNING: Large position error!\n');
        end

    catch ME
        fprintf('        ✗ ERROR: %s\n', ME.message);
        fprintf('        Attempting to continue...\n');
    end

    % Small pause between phases for stability
    pause(0.3);
end

%% ======================== SHUTDOWN ========================

fprintf('\n========================================================\n');
fprintf('  Task 2.c Complete — Returning Home\n');
fprintf('========================================================\n\n');

hw.moveHome();
pause(1);
hw.disconnect();
fprintf('=== Done ===\n');

%% ======================== CLEANUP FUNCTION ========================

function safeShutdown(hw)
    fprintf('\n>>> Emergency cleanup triggered...\n');
    try
        hw.openGripper();
    catch
    end
    try
        hw.disableTorque();
    catch
    end
    try
        hw.disconnect();
    catch
    end
    fprintf('>>> Safe shutdown complete.\n');
end

function [waypoints, phase_labels] = appendManhattanXY(waypoints, phase_labels, start_xy, end_xy, z, pitch, label_id)
    % appendManhattanXY
    % Move in Manhattan style: X-only then Y-only, while Z and pitch stay fixed.
    % Adds intermediate waypoints every MAX_SEG_MM to keep segments short and smooth.
    MAX_SEG_MM = 60;  % Max mm per segment for smooth gate traversal

    sx = start_xy(1); sy = start_xy(2);
    ex = end_xy(1);   ey = end_xy(2);

    % X-only move (Y constant) — add intermediates if segment is long
    if abs(ex - sx) > 1e-9
        dx = ex - sx;
        n_x = max(1, ceil(abs(dx) / MAX_SEG_MM));
        for k = 1:n_x
            t = k / n_x;
            xk = sx + dx * t;
            waypoints(end+1, :) = [xk, sy, z, pitch]; %#ok<AGROW>
            phase_labels(end+1) = label_id; %#ok<AGROW>
        end
    end

    % Y-only move (X constant) — add intermediates if segment is long
    if abs(ey - sy) > 1e-9
        dy = ey - sy;
        n_y = max(1, ceil(abs(dy) / MAX_SEG_MM));
        for k = 1:n_y
            t = k / n_y;
            yk = sy + dy * t;
            waypoints(end+1, :) = [ex, yk, z, pitch]; %#ok<AGROW>
            phase_labels(end+1) = label_id; %#ok<AGROW>
        end
    end

    % No XY movement (still record endpoint marker)
    if abs(ex - sx) <= 1e-9 && abs(ey - sy) <= 1e-9
        waypoints(end+1, :) = [ex, ey, z, pitch]; %#ok<AGROW>
        phase_labels(end+1) = label_id; %#ok<AGROW>
    end
end
