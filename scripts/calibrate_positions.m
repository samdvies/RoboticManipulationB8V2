%% calibrate_positions.m
% Hardware Calibration Diagnostic for OpenManipulator-X
%
% Three structured tests to identify joint zero offsets, link length 
% errors, or mechanical sag:
%
%   Test 1: Zero Config — visually verify home pose
%   Test 2: Single Joint Sweep — isolate per-joint angular offsets
%   Test 3: Position Grid — measure EE error pattern across workspace
%
% Usage (from MATLAB, in project root):
%   run('scripts/calibrate_positions.m')
%
% At each test pose the script PAUSES — measure the physical position
% and press Enter to continue. Type 'q' to skip to next test or quit.

clear; clc;

%% Configuration
COM_PORT    = 'COM4';
BAUDRATE    = 1000000;
VELOCITY    = 15;          % Slow for careful measurement
SETTLE_TIME = 2.0;         % Wait (s) after each move to let arm settle

%% Add source path
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));

%% Connect
fprintf('=============================================\n');
fprintf('  OpenManipulator-X Calibration Diagnostic\n');
fprintf('=============================================\n\n');

hw = OpenManipulator.HardwareInterface(COM_PORT, BAUDRATE);
cleanup = onCleanup(@() safeShutdown(hw));
hw.configure(VELOCITY);
hw.enableTorque();
hw.moveHome();
pause(SETTLE_TIME);

joint_names = {'Base', 'Shoulder', 'Elbow', 'Wrist'};

%% ========================================================================
%  TEST 1: Zero Config
%  ========================================================================
fprintf('\n');
fprintf('##############################################\n');
fprintf('#  TEST 1: ZERO CONFIGURATION                #\n');
fprintf('##############################################\n\n');
fprintf('All joints commanded to 0 degrees.\n');
fprintf('DH convention home pose:\n');
fprintf('  - Shoulder link: straight UP (128mm vertical)\n');
fprintf('  - Elbow link:    horizontal forward (124mm)\n');
fprintf('  - Wrist link:    horizontal forward (126mm)\n\n');

q_home = [0, 0, 0, 0];
hw.moveToAngles(q_home);
pause(SETTLE_TIME);

q_readback = hw.readAngles();
[T_ee, transforms] = OpenManipulator.FK(q_home);
ee_predicted = T_ee(1:3, 4)';

fprintf('Commanded angles:  [%6.1f, %6.1f, %6.1f, %6.1f]°\n', q_home);
fprintf('Readback  angles:  [%6.1f, %6.1f, %6.1f, %6.1f]°\n', q_readback);
fprintf('Angle errors:      [%6.2f, %6.2f, %6.2f, %6.2f]°\n', q_readback - q_home);
fprintf('\nFK predicted EE:   [%.1f, %.1f, %.1f] mm\n', ee_predicted);
fprintf('\n>>> MEASURE: Does the arm match the DH zero pose?\n');
fprintf('>>> Check: Is shoulder link truly vertical?\n');
fprintf('>>> Check: Are elbow/wrist links truly horizontal?\n');
fprintf('>>> Note any angular offset you see for each joint.\n');
waitForUser();

%% ========================================================================
%  TEST 2: Single Joint Sweep
%  ========================================================================
fprintf('\n');
fprintf('##############################################\n');
fprintf('#  TEST 2: SINGLE JOINT SWEEP                #\n');
fprintf('##############################################\n\n');
fprintf('Moves ONE joint at a time in 15° steps.\n');
fprintf('All other joints stay at 0°.\n');
fprintf('At each step, compare physical pose to FK prediction.\n\n');

sweep_angles = [-30, -15, 0, 15, 30];  % degrees

for joint = 1:4
    fprintf('\n--- Joint %d (%s) Sweep ---\n', joint, joint_names{joint});
    fprintf('%8s | %8s | FK Predicted EE (X, Y, Z)\n', 'Command', 'Readback');
    fprintf('%s\n', repmat('-', 1, 55));
    
    skip = false;
    for k = 1:length(sweep_angles)
        q_test = [0, 0, 0, 0];
        q_test(joint) = sweep_angles(k);
        
        % Safety: skip large moves on wrist that could hit the table
        [T_check, ~] = OpenManipulator.FK(q_test);
        if T_check(3, 4) < 10
            fprintf('%7.1f° | SKIPPED (EE too low: Z=%.0fmm)\n', sweep_angles(k), T_check(3,4));
            continue;
        end
        
        % Move home first to decouple, then to target
        hw.moveToAngles([0, 0, 0, 0]);
        pause(0.5);
        hw.moveToAngles(q_test);
        pause(SETTLE_TIME);
        
        q_actual = hw.readAngles();
        [T_ee, ~] = OpenManipulator.FK(q_test);
        ee = T_ee(1:3, 4)';
        
        fprintf('%7.1f° | %7.1f° | [%6.1f, %6.1f, %6.1f]\n', ...
            sweep_angles(k), q_actual(joint), ee(1), ee(2), ee(3));
    end
    
    fprintf('\n>>> MEASURE: Did each physical angle match the commanded value?\n');
    fprintf('>>> If Joint %d consistently reads off by N°, that is the zero offset.\n', joint);
    user_resp = waitForUser();
    if strcmpi(user_resp, 'q')
        fprintf('Skipping remaining joint sweeps.\n');
        break;
    end
end

% Return home
hw.moveToAngles([0, 0, 0, 0]);
pause(SETTLE_TIME);

%% ========================================================================
%  TEST 3: Position Grid
%  ========================================================================
fprintf('\n');
fprintf('##############################################\n');
fprintf('#  TEST 3: POSITION GRID (Y=0, Pitch=0)      #\n');
fprintf('##############################################\n\n');
fprintf('Moves to a grid of (X, Z) positions.\n');
fprintf('At each position, measure the actual EE tip with a ruler.\n');
fprintf('Fill in the "Measured" column.\n\n');

% Grid: X distances at Z=77 (base height), then varying Z at fixed X
grid_targets = [
    % X    Y    Z    Pitch
    150,   0,   77,  0;     % Short reach, base height
    200,   0,   77,  0;     % Medium reach, base height
    250,   0,   77,  0;     % Long reach, base height
    300,   0,   77,  0;     % Far reach, base height
    200,   0,   50,  0;     % Medium reach, below base
    200,   0,  100,  0;     % Medium reach, above base
    200,   0,  150,  0;     % Medium reach, high
    150,   0,   50,  0;     % Short reach, low
    250,   0,  100,  0;     % Long reach, above base
];

n = size(grid_targets, 1);
results = zeros(n, 7);  % [target_x, target_z, fk_x, fk_z, readback_x, readback_z, cmd_err]

fprintf('  # | Target (X, Z) | IK Angles                    | FK Predicted   | Readback FK    | Cmd Err\n');
fprintf('%s\n', repmat('-', 1, 105));

for i = 1:n
    tx = grid_targets(i, 1);
    ty = grid_targets(i, 2);
    tz = grid_targets(i, 3);
    tp = grid_targets(i, 4);
    
    % Compute IK
    try
        q = OpenManipulator.IK(tx, ty, tz, tp);
    catch ME
        fprintf('%3d | (%3.0f, %3.0f)     | UNREACHABLE: %s\n', i, tx, tz, ME.message);
        continue;
    end
    
    % FK on commanded angles
    [T_cmd, ~] = OpenManipulator.FK(q);
    fk_cmd = T_cmd(1:3, 4)';
    
    % Move robot: go home first, then to target
    hw.moveToAngles([0, 0, 0, 0]);
    pause(0.5);
    hw.moveToAnglesInterpolated(q, 10);
    pause(SETTLE_TIME);
    
    % Read actual position
    q_actual = hw.readAngles();
    [T_actual, ~] = OpenManipulator.FK(q_actual);
    fk_actual = T_actual(1:3, 4)';
    
    % Command error = FK(target_q) vs FK(readback_q)
    cmd_err = norm(fk_cmd - fk_actual);
    
    results(i, :) = [tx, tz, fk_cmd(1), fk_cmd(3), fk_actual(1), fk_actual(3), cmd_err];
    
    fprintf('%3d | (%3.0f, %3.0f)     | [%5.1f, %5.1f, %5.1f, %5.1f] | (%5.1f, %5.1f) | (%5.1f, %5.1f) | %4.1fmm\n', ...
        i, tx, tz, q(1), q(2), q(3), q(4), fk_cmd(1), fk_cmd(3), fk_actual(1), fk_actual(3), cmd_err);
end

fprintf('\n');
fprintf('=== MEASUREMENT TABLE ===\n');
fprintf('Fill in the "Measured" column with ruler measurements of physical EE tip.\n\n');
fprintf('  # | Target (X, Z) | FK Predicted   | Measured (X, Z)\n');
fprintf('%s\n', repmat('-', 1, 58));
for i = 1:n
    if results(i, 1) == 0 && results(i, 2) == 0
        continue;
    end
    fprintf('%3d | (%3.0f, %3.0f)     | (%5.1f, %5.1f) | (_____, _____)\n', ...
        i, results(i,1), results(i,2), results(i,3), results(i,4));
end
fprintf('\n');
fprintf('>>> MEASURE each position with a ruler/caliper.\n');
fprintf('>>> Look for patterns:\n');
fprintf('>>>   - Constant offset in X or Z → link length error\n');
fprintf('>>>   - Errors growing with extension → joint zero offset\n');
fprintf('>>>   - Errors only at high extension → mechanical sag\n');
waitForUser();

%% ========================================================================
%  DONE — Return Home
%  ========================================================================
fprintf('\nReturning to HOME...\n');
hw.moveHome();
pause(SETTLE_TIME);
hw.disconnect();
fprintf('\n=== Calibration Diagnostic Complete ===\n');

%% ========================================================================
%  Helper Functions
%  ========================================================================
function resp = waitForUser()
    resp = input('\nPress ENTER to continue (or ''q'' to skip): ', 's');
end

function safeShutdown(hw)
    fprintf('\n>>> Emergency cleanup triggered...\n');
    try hw.disableTorque(); catch, end
    try hw.disconnect(); catch, end
    fprintf('>>> Safe shutdown complete.\n');
end
