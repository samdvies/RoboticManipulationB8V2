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

clear; clc;

%% ======================== CONFIGURATION ========================

% --- Hardware ---
COM_PORT  = '/dev/tty.usbserial-FT9BTEKY';
BAUDRATE  = 1000000;
VELOCITY  = 10;           % Dynamixel profile velocity (~2.3 RPM, slowed down)
DRY_RUN   = false;        % true = IK/FK validation only, no hardware

% --- Motion ---
MOTION_MODE   = 2;        % 1=Joint, 2=Task Linear (best Z constancy), 3=Jacobian
MOVE_TIME     = 4.0;      % Base time per segment (seconds, slowed down)
Z_FLOOR       = 20.0;     % EE Z-floor safety limit (mm)

% --- Task Coordinates (mm) ---
TOOL_PICK_XY  = [-50, -200];     % Tool pickup location (XY)
GRASP_Z       = 70;              % Z to lower for grasping
SAFE_Z        = 85;              % Travel height (above gate tops)
GATE_Z        = 70;              % Z while passing through gates

% Gate waypoints — move through gates in order
GATE_WAYPOINTS = [
    100, -175;    % Waypoint 1
    100,  -75;    % Waypoint 2
    225,  -65;    % Waypoint 3 (Gate entry)
    225,  -35;    % Waypoint 4 (Gate exit)
    175,    0;    % Waypoint 5
    175,  100;    % Waypoint 6
];

DROP_XY  = [50, 100];     % Drop-off location (XY)
DROP_Z   = 50;            % Z for releasing tool

% --- End-Effector Orientation ---
PITCH    = -85;            % Tilted tool (to reach further points like 225, -65)

%% ======================== ADD SOURCE PATH ========================

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

%% ======================== BUILD WAYPOINT LIST ========================
% Each row: [x, y, z, pitch, label_id]
%   label_id: 1=approach, 2=lower_grasp, 3=grip, 4=lift, 5=xy_travel,
%             6=lower_gate, 7=gate_traverse, 8=lift_after, 9=xy_drop,
%             10=lower_drop, 11=release, 12=lift_final

LABELS = {'approach_above_tool', 'lower_to_grasp', 'close_gripper', ...
          'lift_to_safe_z', 'xy_to_gate_entry', 'lower_to_gate_z', ...
          'gate_traverse', 'lift_after_gates', 'xy_to_drop', ...
          'lower_to_drop', 'open_gripper', 'lift_and_home'};

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

% 5. XY travel to first gate entry at SAFE_Z
waypoints(end+1, :) = [GATE_WAYPOINTS(1,:), SAFE_Z, PITCH];
phase_labels(end+1) = 5;

% 6. Lower to gate Z
waypoints(end+1, :) = [GATE_WAYPOINTS(1,:), GATE_Z, PITCH];
phase_labels(end+1) = 6;

% 7. Traverse through all gate waypoints at GATE_Z
for i = 2:size(GATE_WAYPOINTS, 1)
    waypoints(end+1, :) = [GATE_WAYPOINTS(i,:), GATE_Z, PITCH]; %#ok<SAGROW>
    phase_labels(end+1) = 7; %#ok<SAGROW>
end

% 8. Lift after gates
waypoints(end+1, :) = [GATE_WAYPOINTS(end,:), SAFE_Z, PITCH];
phase_labels(end+1) = 8;

% 9. XY travel to drop zone at SAFE_Z
waypoints(end+1, :) = [DROP_XY, SAFE_Z, PITCH];
phase_labels(end+1) = 9;

% 10. Lower to drop Z
waypoints(end+1, :) = [DROP_XY, DROP_Z, PITCH];
phase_labels(end+1) = 10;

% 11. Release (marker waypoint — same position)
waypoints(end+1, :) = [DROP_XY, DROP_Z, PITCH];
phase_labels(end+1) = 11;

% 12. Lift and home
waypoints(end+1, :) = [DROP_XY, SAFE_Z, PITCH];
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
    % Select motion mode: use task-linear (mode 2) for gate segments
    % and XY travel for best Z constancy; joint (mode 1) for lifts/lowers
    if ismember(lid, [5, 7, 9])
        mode = MOTION_MODE;  % XY segments: task-space linear
    else
        mode = 1;            % Vertical movements: joint interpolation
    end

    try
        % Compute IK for logging
        q_target = OpenManipulator.IK(x, y, z, p);
        fprintf('        IK: [%.1f, %.1f, %.1f, %.1f]°\n', q_target);

        % Move
        hw.moveToPose(x, y, z, p, MOVE_TIME, mode, Z_FLOOR);

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
