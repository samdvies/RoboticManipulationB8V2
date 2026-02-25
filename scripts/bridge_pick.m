%% bridge_pick.m
% Pick an object from under a bridge using the proven safe waypoint path.
%
% Strategy (mirrors the Python simulator's verified path):
%   1. Start at HOME (clear of bridge)
%   2. Half-shut gripper to fit through bridge gap
%   3. Lift to safe Z above bridge deck
%   4. Sweep to approach X (just in front of bridge)
%   5. Drop to lowest reachable Z at approach X
%   6. Slide diagonally to pick position (through bridge gap)
%   7. Close gripper
%   8. Reverse path to exit
%
% Bridge specs: 25mm X-width, 50mm Y-gap between pillars, 60mm tall.
%
% Usage:
%   run('scripts/bridge_pick.m')

clc; clear;
addpath(genpath('../src'));

% --- Hardware Configuration ---
PORT      = 'COM4';
BAUD      = 1000000;
VELOCITY  = 20;
MOVE_TIME = 2.5;     % seconds per standard move
SLIDE_TIME = 3.0;    % seconds for diagonal slide (slower for precision)
Z_FLOOR   = 10;      % mm, minimum EE Z
MOTION_MODE = 2;     % 2 = Task-space linear (straight line)

% --- Bridge Geometry (real specs) ---
BRIDGE_X_MIN = 217.5;    % mm
BRIDGE_X_MAX = 232.5;    % mm
BRIDGE_Z_MAX = 85;       % mm (deck top)
BRIDGE_GAP_Y = 50;       % mm (gap between pillars)

% --- Derived Approach Parameters ---
APPROACH_X  = BRIDGE_X_MIN - 12;   % just in front of bridge (~175.5mm)
SAFE_Z      = BRIDGE_Z_MAX + 30;   % above deck top (~90mm)
DROP_Z      = 48;                   % lowest reachable Z at approach X

% --- Default Positions ---
home_pose = [200, 0, 180, 0];     % safe starting position
def_pick  = [225, 0, 30];         % default pick coordinates

fprintf('==========================================\n');
fprintf('  Bridge Pick (Hardware)\n');
fprintf('==========================================\n');
fprintf('  Bridge: X=[%.1f, %.1f] Z=[0, %d]\n', BRIDGE_X_MIN, BRIDGE_X_MAX, BRIDGE_Z_MAX);
fprintf('  Approach X: %.1f  Safe Z: %d  Drop Z: %d\n', APPROACH_X, SAFE_Z, DROP_Z);
fprintf('==========================================\n\n');

% --- Get Pick Coordinates ---
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

pick_pitch = 0;  % horizontal approach

fprintf('\n  Pick target: [%.0f, %.0f, %.0f] pitch=%.0f\n\n', ...
    pick_x, pick_y, pick_z, pick_pitch);

% --- Waypoint Path (mirrors Python exit/entry path) ---
% Entry waypoints: home -> lift -> approach -> drop -> slide to pick
wp_entry = [
    home_pose(1), home_pose(2), SAFE_Z,  0;   % 1. Lift to safe Z at current X
    APPROACH_X,   pick_y,       SAFE_Z,  0;   % 2. Sweep to approach X above bridge
    APPROACH_X,   pick_y,       DROP_Z,  0;   % 3. Drop to reachable Z
    pick_x,       pick_y,       pick_z,  0;   % 4. Slide diagonally to pick
];

% Exit waypoints: pick -> slide out -> lift -> home
wp_exit = [
    APPROACH_X,   pick_y,       DROP_Z,  0;   % 1. Slide out to approach X
    APPROACH_X,   pick_y,       SAFE_Z,  0;   % 2. Lift above bridge
    home_pose(1), home_pose(2), SAFE_Z,  0;   % 3. Sweep back
    home_pose;                                 % 4. Return home
];

% --- Reachability Check ---
fprintf('Checking waypoint reachability...\n');
check_points = {
    'Home',     home_pose;
    'Lift',     wp_entry(1,:);
    'Approach', wp_entry(2,:);
    'Drop',     wp_entry(3,:);
    'Pick',     wp_entry(4,:);
};
all_ok = true;
for i = 1:size(check_points, 1)
    name = check_points{i,1};
    p = check_points{i,2};
    try
        q = OpenManipulator.IK(p(1), p(2), p(3), p(4));
        [T, ~] = OpenManipulator.FK(q);
        ee = T(1:3,4)';
        err = norm(ee - p(1:3));
        fprintf('  %-10s [%6.1f, %5.1f, %5.1f] -> q=[%6.1f,%6.1f,%6.1f,%6.1f] err=%.1fmm\n', ...
            name, p(1), p(2), p(3), q(1), q(2), q(3), q(4), err);
    catch ME
        fprintf('  %-10s [%6.1f, %5.1f, %5.1f] -> UNREACHABLE: %s\n', ...
            name, p(1), p(2), p(3), ME.message);
        all_ok = false;
    end
end
if ~all_ok
    fprintf('\n*** Some positions are unreachable! Adjust coordinates. ***\n');
    return;
end

fprintf('\nAll positions reachable.\n');
input('Press ENTER to connect and start...', 's');

try
    % --- Connect & Configure ---
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    cleanup = onCleanup(@() safeShutdown(hw));
    hw.configure(VELOCITY);
    hw.enableTorque();

    % ===== PHASE 0: HOME + HALF-SHUT GRIPPER =====
    fprintf('\n[PHASE 0] Moving HOME and setting gripper...\n');
    hw.moveToPose(home_pose(1), home_pose(2), home_pose(3), home_pose(4), ...
        MOVE_TIME, 1, Z_FLOOR);
    pause(0.5);
    
    % Half-shut gripper to fit through bridge gap (50% = ~20mm jaw width)
    fprintf('  Setting gripper to HALF-SHUT (50%%)...\n');
    hw.setGripperPosition(50);
    pause(1);

    % ===== PHASE 1: ENTRY WAYPOINTS =====
    wp_names = {'LIFT above bridge', 'SWEEP to approach X', ...
                'DROP to reachable Z', 'SLIDE to pick position'};
    
    for i = 1:size(wp_entry, 1)
        wp = wp_entry(i, :);
        fprintf('[PHASE %d] %s -> [%.0f, %.0f, %.0f]...\n', ...
            i, wp_names{i}, wp(1), wp(2), wp(3));
        
        if i == 4
            % Final slide: use slower speed for precision
            hw.moveToPose(wp(1), wp(2), wp(3), wp(4), ...
                SLIDE_TIME, MOTION_MODE, Z_FLOOR);
        else
            hw.moveToPose(wp(1), wp(2), wp(3), wp(4), ...
                MOVE_TIME, MOTION_MODE, Z_FLOOR);
        end
        pause(0.5);
    end

    fprintf('\n  === AT PICK POSITION [%.0f, %.0f, %.0f] ===\n', ...
        pick_x, pick_y, pick_z);

    % ===== PHASE 5: GRIP =====
    choice = input('  Close gripper to pick? (y/n): ', 's');
    if strcmpi(choice, 'y')
        fprintf('[PHASE 5] Closing gripper...\n');
        hw.closeGripper();
        pause(2);
        fprintf('  Object gripped.\n');
    else
        fprintf('  Skipping grip.\n');
    end

    % ===== PHASE 5.5: LIFT 7.5mm STRAIGHT UP =====
    fprintf('[PHASE 5.5] Lifting 7.5mm straight up...\n');
    hw.moveToPose(pick_x, pick_y, pick_z + 7.5, pick_pitch, ...
        MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    % ===== PHASE 6: EXIT WAYPOINTS =====
    input('Press ENTER to retract from bridge...', 's');
    
    exit_names = {'SLIDE OUT to approach X', 'LIFT above bridge', ...
                  'SWEEP back', 'Return HOME'};
    
    for i = 1:size(wp_exit, 1)
        wp = wp_exit(i, :);
        fprintf('[PHASE %d] %s -> [%.0f, %.0f, %.0f]...\n', ...
            i+5, exit_names{i}, wp(1), wp(2), wp(3));
        
        if i == 1
            % First retract: slow for safety
            hw.moveToPose(wp(1), wp(2), wp(3), wp(4), ...
                SLIDE_TIME, MOTION_MODE, Z_FLOOR);
        else
            hw.moveToPose(wp(1), wp(2), wp(3), wp(4), ...
                MOVE_TIME, MOTION_MODE, Z_FLOOR);
        end
        pause(0.5);
    end

    % ===== DONE =====
    fprintf('\n  Opening gripper...\n');
    hw.openGripper();
    pause(0.5);
    hw.disconnect();
    fprintf('\n=== Bridge Pick Complete ===\n');

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
