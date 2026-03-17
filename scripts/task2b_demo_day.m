%% TASK2B_DEMO_DAY  Demo Day Task 2B — Tuneable 3-Cube Pick & Stack
%
% Usage:
%   run('scripts/task2b_demo_day.m')
%
% Layout:
%   C1 (0,-100)  →  stack at (175,0) level 0
%   C2 (100,-200) →  stack at (175,0) level 1
%   C3 (100, 100) →  stack at (175,0) level 2
%
% All cubes: standard vertical pick (pitch=-90), place (pitch=-90).
% No rotation, no bridge avoidance.

clc; clear;
script_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(script_dir, '..', 'src')));

% #########################################################################
%  >>> TUNING PARAMETERS — EDIT THESE FOR CALIBRATION <<<
% #########################################################################

% ======================== WHICH CUBES TO RUN =============================
RUN_CUBE1 = true;
RUN_CUBE2 = true;
RUN_CUBE3 = true;

TUNING_MODE = true;   % after each place, prompt for offset tweaks & re-run

% ======================== HARDWARE =======================================
PORT         = 'COM4';
BAUD         = 1000000;
VELOCITY     = 20;
MOVE_TIME    = 1.25;
PICK_HOLD_TIME = 2.5;
PICK_LIFT_MM   = 5;
Z_FLOOR      = 15;
MOTION_MODE  = 1;       % 1=Joint, 2=Task-space, 3=Jacobian

% ======================== CUBE GEOMETRY ==================================
CUBE_Z_SURFACE = 32.5;
CUBE_SIZE      = 25;
PICK_Z_BASE    = CUBE_Z_SURFACE + CUBE_SIZE/2;   % 45 mm

% ======================== POSITIONS ======================================
CUBE1_XY  = [  0, -100];
CUBE2_XY  = [100, -200];
CUBE3_XY  = [100,  100];
TARGET_XY = [175,    0];

% ======================== PER-CUBE PICK OFFSETS  [dx dy dz] ==============
C1_PICK_OFFSET  = [0, 0, 0];
C2_PICK_OFFSET  = [0, 0, 0];
C3_PICK_OFFSET  = [0, 0, 0];

% ======================== PER-CUBE PLACE OFFSETS [dx dy dz] ==============
C1_PLACE_OFFSET = [0, 0, 0];
C2_PLACE_OFFSET = [0, 0, 0];
C3_PLACE_OFFSET = [0, 0, 0];

% ======================== GLOBAL PLACE OFFSET ============================
PLACE_OFFSET_MAG = sqrt(32);   % ~5.66 mm nudge toward origin

% ======================== PITCH (all standard vertical) ==================
PICK_PITCH  = -90;
PLACE_PITCH = -90;

% ======================== HEIGHTS / TRANSIT ==============================
HOVER_Z               = CUBE_Z_SURFACE + 80;    % 112.5 mm
PLACE_VERTICAL_OFFSET = 25;                      % mm above stack for approach
HOME_POSE             = [134, 0, 240, -45];

% #########################################################################
%  END OF TUNING PARAMETERS
% #########################################################################

PLACE_Z = @(level) PICK_Z_BASE + level * CUBE_SIZE;

% Print summary
fprintf('==========================================================\n');
fprintf('  Task 2B — Demo Day   (3-cube stack, no rotation/bridge)\n');
fprintf('==========================================================\n');
fprintf('  C1: (%+.0f,%+.0f) pick_off(%+.0f,%+.0f,%+.0f) %s\n', CUBE1_XY, C1_PICK_OFFSET, onoff(RUN_CUBE1));
fprintf('  C2: (%+.0f,%+.0f) pick_off(%+.0f,%+.0f,%+.0f) %s\n', CUBE2_XY, C2_PICK_OFFSET, onoff(RUN_CUBE2));
fprintf('  C3: (%+.0f,%+.0f) pick_off(%+.0f,%+.0f,%+.0f) %s\n', CUBE3_XY, C3_PICK_OFFSET, onoff(RUN_CUBE3));
fprintf('  Target: (%+.0f,%+.0f)  Stack Z: %.0f | %.0f | %.0f mm\n', ...
    TARGET_XY, PLACE_Z(0), PLACE_Z(1), PLACE_Z(2));
fprintf('  Tuning: %s\n', onoff(TUNING_MODE));
fprintf('==========================================================\n\n');

inp = input('Press ENTER to connect and start (or ''n'' to abort): ', 's');
if strcmpi(strtrim(inp), 'n'), fprintf('Aborted.\n'); return; end

try
    fprintf('\n[INIT] Connecting %s...\n', PORT);
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    cleanup = onCleanup(@() safeShutdown(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR));
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(0.5);
    goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR);

    % Cube data: {name, nominal_xy, pick_offset, place_offset, stack_level, run_flag}
    cubes = {
        'C1', CUBE1_XY, C1_PICK_OFFSET, C1_PLACE_OFFSET, 0, RUN_CUBE1;
        'C2', CUBE2_XY, C2_PICK_OFFSET, C2_PLACE_OFFSET, 1, RUN_CUBE2;
        'C3', CUBE3_XY, C3_PICK_OFFSET, C3_PLACE_OFFSET, 2, RUN_CUBE3;
    };

    for ci = 1:size(cubes, 1)
        label     = cubes{ci, 1};
        nom_xy    = cubes{ci, 2};
        pick_off  = cubes{ci, 3};
        place_off = cubes{ci, 4};
        slevel    = cubes{ci, 5};
        do_run    = cubes{ci, 6};

        if ~do_run, continue; end

        cube_done = false;
        while ~cube_done
            fprintf('\n========== %s — Pick & Stack (level %d) ==========\n', label, slevel);

            % Compute pick position with offset
            cpx = nom_xy(1) + pick_off(1);
            cpy = nom_xy(2) + pick_off(2);
            cpz = PICK_Z_BASE + pick_off(3);

            % Compute place position with offset
            [ppx, ppy] = computePlaceXY(TARGET_XY(1), TARGET_XY(2), PLACE_OFFSET_MAG, place_off);
            ppz = PLACE_Z(slevel) + place_off(3);

            fprintf('  Pick:  (%.1f, %.1f, %.1f)\n', cpx, cpy, cpz);
            fprintf('  Place: (%.1f, %.1f, %.1f)  level %d\n', ppx, ppy, ppz, slevel);

            % ── PICK ─────────────────────────────────────────────────────
            hw.openGripper(); pause(0.3);

            fprintf('[%s.1] Above cube...\n', label);
            hw.moveToPose(cpx, cpy, HOVER_Z, 0, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[%s.2] Tilt pitch=%+.0f...\n', label, PICK_PITCH);
            hw.moveToPose(cpx, cpy, HOVER_Z, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[%s.3] Descend z=%.1f...\n', label, cpz);
            hw.moveToPose(cpx, cpy, cpz, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            fprintf('[%s.4] Grip...\n', label);
            hw.closeGripper(); pause(PICK_HOLD_TIME);

            fprintf('[%s.5] Micro-lift +%d mm...\n', label, PICK_LIFT_MM);
            hw.moveToPose(cpx, cpy, cpz + PICK_LIFT_MM, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[%s.6] Lift to hover...\n', label);
            hw.moveToPose(cpx, cpy, HOVER_Z, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            % ── PLACE ────────────────────────────────────────────────────
            fprintf('[%s.7] Fly to target...\n', label);
            hw.moveToPose(ppx, ppy, HOVER_Z, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[%s.8] Approach z=%.1f...\n', label, ppz + PLACE_VERTICAL_OFFSET);
            hw.moveToPose(ppx, ppy, ppz + PLACE_VERTICAL_OFFSET, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[%s.9] Lower z=%.1f...\n', label, ppz);
            hw.moveToPose(ppx, ppy, ppz, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            fprintf('[%s.10] Release...\n', label);
            hw.openGripper(); pause(1.0);

            fprintf('[%s.11] Retract...\n', label);
            hw.moveToPose(ppx, ppy, HOVER_Z, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);
            goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            fprintf('=== %s placed (level %d) ===\n', label, slevel);

            % ── TUNING ───────────────────────────────────────────────────
            cube_done = true;
            if TUNING_MODE
                [cube_done, pick_off, place_off] = tuningPrompt(label, pick_off, place_off);
                cubes{ci, 3} = pick_off;
                cubes{ci, 4} = place_off;
            end
        end
    end

    %% DONE
    fprintf('\n==========================================================\n');
    fprintf('  Task 2B COMPLETE — 3 cubes stacked at (%+.0f, %+.0f)\n', TARGET_XY);
    fprintf('  Final offsets:\n');
    for ci = 1:size(cubes, 1)
        fprintf('    %s pick(%+.1f,%+.1f,%+.1f) place(%+.1f,%+.1f,%+.1f)\n', ...
            cubes{ci,1}, cubes{ci,3}, cubes{ci,4});
    end
    fprintf('==========================================================\n');
    hw.disconnect();

catch ME
    fprintf('\nERROR: %s\n', ME.message);
    fprintf('Stack:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s  line %d\n', ME.stack(k).name, ME.stack(k).line);
    end
end


%% ── HELPERS ─────────────────────────────────────────────────────────────

function [px, py] = computePlaceXY(tx, ty, mag, extra)
    r = sqrt(tx^2 + ty^2);
    if r > 1e-6
        px = tx - mag * tx / r + extra(1);
        py = ty - mag * ty / r + extra(2);
    else
        px = tx + extra(1);  py = ty + extra(2);
    end
end

function goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR)
    fprintf('[HOME]\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);
end

function s = onoff(flag)
    if flag, s = 'ON'; else, s = 'OFF'; end
end

function [done, pick_off, place_off] = tuningPrompt(label, pick_off, place_off)
    fprintf('\n--- %s Tuning ---\n', label);
    fprintf('  pick  offset: (%+.1f, %+.1f, %+.1f)\n', pick_off);
    fprintf('  place offset: (%+.1f, %+.1f, %+.1f)\n', place_off);
    fprintf('  Enter new values or Enter to keep, ''n'' to accept & move on.\n');
    inp = input(sprintf('  %s pick  dX dY dZ [%+.1f %+.1f %+.1f]: ', label, pick_off), 's');
    if strcmpi(strtrim(inp), 'n'), done = true; return; end
    if ~isempty(strtrim(inp))
        v = str2num(inp); %#ok<ST2NM>
        if numel(v) >= 3, pick_off = v(1:3); elseif numel(v) >= 2, pick_off(1:2) = v(1:2); end
    end
    inp = input(sprintf('  %s place dX dY dZ [%+.1f %+.1f %+.1f]: ', label, place_off), 's');
    if strcmpi(strtrim(inp), 'n'), done = true; return; end
    if ~isempty(strtrim(inp))
        v = str2num(inp); %#ok<ST2NM>
        if numel(v) >= 3, place_off = v(1:3); elseif numel(v) >= 2, place_off(1:2) = v(1:2); end
    end
    fprintf('  Updated: pick(%+.1f,%+.1f,%+.1f) place(%+.1f,%+.1f,%+.1f)\n', pick_off, place_off);
    fprintf('  Re-running %s...\n\n', label);
    done = false;
end

function safeShutdown(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR)
    fprintf('\n>>> Emergency cleanup...\n');
    try hw.openGripper(); catch, end
    try hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR); catch, end
    try hw.disconnect(); catch, end
    fprintf('>>> Done.\n');
end
