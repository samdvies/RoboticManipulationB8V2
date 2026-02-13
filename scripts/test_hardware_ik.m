%% test_hardware_ik.m
% Interactive Hardware IK Test for OpenManipulator-X
%
% Initializes with safe velocity (20), moves to home, then loops
% accepting [x, y, z] coordinates and moving the real robot.
%
% Usage (from MATLAB, in project root):
%   run('scripts/test_hardware_ik.m')
%
% Controls:
%   Enter [x, y, z] as comma-separated values: 200, 0, 100
%   Type 'q' to quit safely
%   Ctrl+C triggers emergency stop via onCleanup

clear;
clc;

%% Configuration
COM_PORT = 'COM4';
BAUDRATE = 1000000;
SAFE_VELOCITY = 20;       % ~4.6 RPM — very slow for testing
INTERP_STEPS = 10;        % Linear interpolation steps per move
DEFAULT_PITCH = 0;        % Horizontal EE orientation

%% Add source path
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

%% Initialize Hardware
fprintf('========================================\n');
fprintf('   OpenManipulator-X IK Hardware Test\n');
fprintf('========================================\n\n');

hw = OpenManipulator.HardwareInterface(COM_PORT, BAUDRATE);

% Setup emergency stop on Ctrl+C or error
cleanup = onCleanup(@() safeShutdown(hw));

% Configure with safe velocity
hw.configure(SAFE_VELOCITY);

% Enable torque and move to home
hw.enableTorque();
fprintf('\nMoving to HOME (all joints to 0°)...\n');
hw.moveHome();

fprintf('\n--- Ready for commands ---\n');
fprintf('Enter target as: x, y, z  (mm)\n');
fprintf('Example: 200, 0, 100\n');
fprintf('Gripper: ''open'', ''close'', or ''grip 50'' (0-100%%)\n');
fprintf('Type ''q'' to quit.\n\n');

%% Interactive Loop
while true
    user_input = input('Command: ', 's');
    trimmed = strtrim(user_input);

    % Check for quit
    if strcmpi(trimmed, 'q')
        fprintf('\nQuitting...\n');
        break;
    end

    % --- Gripper commands ---
    if strcmpi(trimmed, 'open')
        hw.openGripper();
        pause(1);
        fprintf('Gripper position: %.0f%%\n\n', hw.readGripperPosition());
        continue;
    end

    if strcmpi(trimmed, 'close')
        hw.closeGripper();
        pause(1);
        fprintf('Gripper position: %.0f%%\n\n', hw.readGripperPosition());
        continue;
    end

    if strncmpi(trimmed, 'grip', 4)
        tokens = strsplit(trimmed);
        if length(tokens) >= 2
            pct = str2double(tokens{2});
            if ~isnan(pct)
                hw.setGripperPosition(pct);
                pause(1);
                fprintf('Gripper position: %.0f%%\n\n', hw.readGripperPosition());
            else
                fprintf('ERROR: Use format: grip 50\n\n');
            end
        else
            fprintf('ERROR: Use format: grip 50\n\n');
        end
        continue;
    end

    % --- Arm movement (x, y, z) ---
    try
        vals = str2double(strsplit(trimmed, {',', ' '}));
        vals = vals(~isnan(vals));

        if length(vals) ~= 3
            fprintf('ERROR: Enter 3 values (x, y, z), or a gripper command.\n\n');
            continue;
        end

        x = vals(1);
        y = vals(2);
        z = vals(3);
    catch
        fprintf('ERROR: Could not parse input. Use format: 200, 0, 100\n\n');
        continue;
    end

    % Compute IK
    fprintf('\nTarget: [%.1f, %.1f, %.1f] mm, Pitch: %.1f°\n', x, y, z, DEFAULT_PITCH);

    try
        q = OpenManipulator.IK(x, y, z, DEFAULT_PITCH);
        fprintf('IK Solution: [%.1f, %.1f, %.1f, %.1f]°\n', q);

        % Validate with FK
        [T_ee, ~] = OpenManipulator.FK(q);
        fk_pos = T_ee(1:3, 4)';
        fprintf('FK Verify:   [%.1f, %.1f, %.1f] mm\n', fk_pos);
        fk_err = norm(fk_pos - [x, y, z]);
        fprintf('Position Error: %.1f mm\n', fk_err);

        if fk_err > 10
            fprintf('WARNING: Large FK error — target may be near workspace boundary.\n');
        end

        % Move robot with interpolation
        fprintf('Moving robot (interpolated, %d steps)...\n', INTERP_STEPS);
        hw.moveToAnglesInterpolated(q, INTERP_STEPS);

        % Read actual position
        q_actual = hw.readAngles();
        fprintf('Actual Joints: [%.1f, %.1f, %.1f, %.1f]°\n', q_actual);

        % FK on actual
        [T_actual, ~] = OpenManipulator.FK(q_actual);
        actual_pos = T_actual(1:3, 4)';
        fprintf('Actual Pos:    [%.1f, %.1f, %.1f] mm\n', actual_pos);
        fprintf('Hardware Error: %.1f mm\n', norm(actual_pos - [x, y, z]));

        % Show gripper status
        fprintf('Gripper: %.0f%%\n\n', hw.readGripperPosition());

    catch ME
        fprintf('ERROR: %s\n\n', ME.message);
    end
end

%% Shutdown
fprintf('\nReturning to HOME...\n');
hw.moveHome();
hw.disconnect();
fprintf('\n=== Test Complete ===\n');

%% Cleanup Function
function safeShutdown(hw)
    fprintf('\n>>> Emergency cleanup triggered...\n');
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
