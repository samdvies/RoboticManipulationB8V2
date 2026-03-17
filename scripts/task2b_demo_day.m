%% TASK2B_DEMO_DAY  Demo Day Task 2B — Tuneable 3-Cube Stacking
%
% Usage:
%   run('scripts/task2b_demo_day.m')
%
% Layout (demo day):
%   C1 (0,-100)    red face +Y → grip pitch=-90, rotate to 0 → red-up
%   C2 (100,-200)  under bridge (X 50→150, Y≈-200), pitch=0 approach
%   C3 (100, 100)  standard vertical pick
%   Target (175,0) stack all 3, bottom→top: C1, C2, C3
%
% Tunability:
%   - Per-cube pick/place XYZ offsets at the top
%   - Per-cube pitch overrides
%   - RUN_CUBE1/2/3 flags (skip cubes already placed)
%   - TUNING_MODE: after each place, prompt for offset corrections & re-run
%   - Bridge geometry fully configurable
%   - All timing/speed at the top

clc; clear;
script_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(script_dir, '..', 'src')));

% #########################################################################
%  >>> TUNING PARAMETERS — EDIT THESE FOR CALIBRATION <<<
% #########################################################################

% ======================== WHICH CUBES TO RUN =============================
RUN_CUBE1 = true;     % set false to skip Cube 1 (e.g. already placed)
RUN_CUBE2 = true;     % set false to skip Cube 2
RUN_CUBE3 = true;     % set false to skip Cube 3

TUNING_MODE = true;   % true = after each place, prompt for offset tweaks
                       %        and offer to re-run that same cube

% ======================== HARDWARE =======================================
PORT         = 'COM4';
BAUD         = 1000000;
VELOCITY     = 20;          % joint velocity (lower = slower/safer)
MOVE_TIME    = 1.25;        % seconds per waypoint move
PICK_HOLD_TIME = 2.5;       % gripper hold after close (s)
PICK_LIFT_MM   = 5;         % micro-lift after grip (mm)
Z_FLOOR      = 15;          % safety floor (mm)
MOTION_MODE  = 1;           % 1=Joint, 2=Task-space, 3=Jacobian

% ======================== CUBE GEOMETRY ==================================
CUBE_Z_SURFACE = 32.5;      % table-to-cube-bottom (mm)
CUBE_SIZE      = 25;         % cube side (mm)
PICK_Z_BASE    = CUBE_Z_SURFACE + CUBE_SIZE/2;  % = 45 mm, cube centre

% ======================== POSITIONS (nominal) ============================
CUBE1_XY  = [  0, -100];    % Cube 1 pick position
CUBE2_XY  = [100, -200];    % Cube 2 pick position (under bridge)
CUBE3_XY  = [100,  100];    % Cube 3 pick position

TARGET_XY = [175,    0];    % Stack target position

% ======================== PER-CUBE PICK OFFSETS ==========================
%  [dx, dy, dz] mm — added to nominal pick position
%  Positive dx = further in +X, dy = further in +Y, dz = higher
C1_PICK_OFFSET  = [0, 0, 0];
C2_PICK_OFFSET  = [0, 0, 0];
C3_PICK_OFFSET  = [0, 0, 0];

% ======================== PER-CUBE PLACE OFFSETS =========================
%  [dx, dy, dz] mm — added to computed place position
%  These are ON TOP of the global PLACE_OFFSET_MAG nudge toward origin
C1_PLACE_OFFSET = [0, 0, 0];
C2_PLACE_OFFSET = [0, 0, 0];
C3_PLACE_OFFSET = [0, 0, 0];

% ======================== GLOBAL PLACE OFFSET ============================
%  Magnitude (mm) nudged toward origin to compensate arm flex
PLACE_OFFSET_MAG = sqrt(32);   % ≈5.66 mm

% ======================== PER-CUBE PITCH =================================
%  Override pick/place pitch per cube if needed
C1_PICK_PITCH  = -90;   C1_PLACE_PITCH =   0;   % rotation pick
C2_PICK_PITCH  =   0;   C2_PLACE_PITCH =   0;   % bridge (horizontal)
C3_PICK_PITCH  = -90;   C3_PLACE_PITCH = -90;   % standard vertical

% ======================== HEIGHTS / TRANSIT ==============================
HOVER_Z              = CUBE_Z_SURFACE + 80;    % 112.5 mm hover
PLACE_VERTICAL_OFFSET = 25;                     % straight-down approach mm
TRANSIT_PITCH        = -45;                     % pitch for high-Z transit
MIN_ROTATE_RADIUS    = 200;                     % pullout radius for rotation
HOME_POSE            = [134, 0, 240, -45];      % home [x y z pitch]

% ======================== BRIDGE GEOMETRY ================================
%  Bridge deck runs along X from X_MIN to X_MAX at Y ≈ Y_CENTER
BRIDGE_X_MIN       =  50;
BRIDGE_X_MAX       = 150;
BRIDGE_Y_CENTER    = -200;
BRIDGE_Y_HALF_GAP  =  25;     % half of gap between pillars (mm)
BRIDGE_PILLAR_W    =  10;     % pillar width in Y (mm)
BRIDGE_DECK_THICK  =  10;     % deck thickness (mm)
BRIDGE_Z_MIN       =  90;
BRIDGE_Z_MAX       = 105;

% Planner X tuning
X_SCAN_MIN       = 80.0;
X_FRONT_OFFSET   = 30.0;
X_RETREAT_OFFSET = 40.0;

% #########################################################################
%  END OF TUNING PARAMETERS
% #########################################################################

% ======================== DERIVED ========================================
% Stack heights:  level 0 → 45mm,  level 1 → 70mm,  level 2 → 95mm
PLACE_Z = @(level) PICK_Z_BASE + level * CUBE_SIZE;

% ======================== PRINT SUMMARY ==================================
fprintf('==========================================================\n');
fprintf('  Task 2B — Demo Day   (tuneable 3-cube stack)\n');
fprintf('==========================================================\n');
fprintf('  Cube 1: pick (%+.0f,%+.0f) offset(%+.0f,%+.0f,%+.0f) pitch %+.0f→%+.0f  %s\n', ...
    CUBE1_XY, C1_PICK_OFFSET, C1_PICK_PITCH, C1_PLACE_PITCH, onoff(RUN_CUBE1));
fprintf('  Cube 2: pick (%+.0f,%+.0f) offset(%+.0f,%+.0f,%+.0f) pitch %+.0f→%+.0f  %s [BRIDGE]\n', ...
    CUBE2_XY, C2_PICK_OFFSET, C2_PICK_PITCH, C2_PLACE_PITCH, onoff(RUN_CUBE2));
fprintf('  Cube 3: pick (%+.0f,%+.0f) offset(%+.0f,%+.0f,%+.0f) pitch %+.0f→%+.0f  %s\n', ...
    CUBE3_XY, C3_PICK_OFFSET, C3_PICK_PITCH, C3_PLACE_PITCH, onoff(RUN_CUBE3));
fprintf('  Target: (%+.0f,%+.0f)  Place offsets: C1(%+.0f,%+.0f,%+.0f) C2(%+.0f,%+.0f,%+.0f) C3(%+.0f,%+.0f,%+.0f)\n', ...
    TARGET_XY, C1_PLACE_OFFSET, C2_PLACE_OFFSET, C3_PLACE_OFFSET);
fprintf('  Stack Z: %.0f | %.0f | %.0f mm   Tuning: %s\n', ...
    PLACE_Z(0), PLACE_Z(1), PLACE_Z(2), onoff(TUNING_MODE));
fprintf('  Bridge: X=[%d,%d]  Y_center=%d  gap=±%d\n', ...
    BRIDGE_X_MIN, BRIDGE_X_MAX, BRIDGE_Y_CENTER, BRIDGE_Y_HALF_GAP);
fprintf('==========================================================\n\n');

% ======================== BRIDGE ZONES ===================================
bridge_zone = OpenManipulator.BridgeAvoidance.NewZone( ...
    BRIDGE_X_MIN, BRIDGE_X_MAX, ...
    BRIDGE_Y_CENTER - BRIDGE_Y_HALF_GAP - BRIDGE_PILLAR_W, ...
    BRIDGE_Y_CENTER + BRIDGE_Y_HALF_GAP + BRIDGE_PILLAR_W, ...
    BRIDGE_Z_MIN, BRIDGE_Z_MAX);
bridge_zones = OpenManipulator.BridgeAvoidance.BuildBridgeZones( ...
    bridge_zone, BRIDGE_Y_HALF_GAP * 2, BRIDGE_PILLAR_W, BRIDGE_DECK_THICK);

planner_opts = struct( ...
    'pitch_tolerance_deg',    5.0, ...
    'vertical_clearance_mm', 30.0, ...
    'samples',                 20, ...
    'x_scan_min',       X_SCAN_MIN, ...
    'x_front_offset',   X_FRONT_OFFSET, ...
    'x_retreat_offset', X_RETREAT_OFFSET);

% ======================== CONFIRM ========================================
inp = input('Press ENTER to connect and start (or ''n'' to abort): ', 's');
if strcmpi(strtrim(inp), 'n'), fprintf('Aborted.\n'); return; end

try
    %% ── CONNECT ─────────────────────────────────────────────────────────
    fprintf('\n[INIT] Connecting %s @ %d...\n', PORT, BAUD);
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    cleanup = onCleanup(@() safeShutdown(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR));
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(0.5);

    goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR);

    % =====================================================================
    %  CUBE 1 — rotation pick (red face +Y → red-up)
    %  Stack level 0
    % =====================================================================
    if RUN_CUBE1
        cube1_done = false;
        while ~cube1_done
            fprintf('\n========== CUBE 1 — Rotation pick ==========\n');

            % Compute pick XY with offsets
            cpx1 = CUBE1_XY(1) + C1_PICK_OFFSET(1);
            cpy1 = CUBE1_XY(2) + C1_PICK_OFFSET(2);
            cpz1 = PICK_Z_BASE + C1_PICK_OFFSET(3);

            % Compute place XY
            [ppx1, ppy1] = computePlaceXY(TARGET_XY(1), TARGET_XY(2), PLACE_OFFSET_MAG, C1_PLACE_OFFSET);
            ppz1 = PLACE_Z(0) + C1_PLACE_OFFSET(3);

            fprintf('  Pick:  (%.1f, %.1f, %.1f) pitch=%+.0f\n', cpx1, cpy1, cpz1, C1_PICK_PITCH);
            fprintf('  Place: (%.1f, %.1f, %.1f) pitch=%+.0f\n', ppx1, ppy1, ppz1, C1_PLACE_PITCH);

            % Pullout radius check (need safe radius for rotation)
            r1 = sqrt(cpx1^2 + cpy1^2);
            needs_pullout = (r1 < MIN_ROTATE_RADIUS && r1 > 0.001);
            if needs_pullout
                sc = MIN_ROTATE_RADIUS / r1;
                safe_x = cpx1 * sc;  safe_y = cpy1 * sc;
            else
                safe_x = cpx1;  safe_y = cpy1;
            end

            % Pick sequence
            hw.openGripper(); pause(0.3);
            fprintf('[C1.1] Above cube...\n');
            hw.moveToPose(cpx1, cpy1, HOVER_Z, 0, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C1.2] Tilt to grip pitch=%+.0f...\n', C1_PICK_PITCH);
            hw.moveToPose(cpx1, cpy1, HOVER_Z, C1_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C1.3] Descend z=%.1f...\n', cpz1);
            hw.moveToPose(cpx1, cpy1, cpz1, C1_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            fprintf('[C1.4] Grip...\n');
            hw.closeGripper(); pause(PICK_HOLD_TIME);

            fprintf('[C1.5] Micro-lift +%d mm...\n', PICK_LIFT_MM);
            hw.moveToPose(cpx1, cpy1, cpz1 + PICK_LIFT_MM, C1_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C1.6] Lift to hover...\n');
            hw.moveToPose(cpx1, cpy1, HOVER_Z, C1_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            if needs_pullout
                fprintf('[C1.7] Pull to safe radius (%.0f,%.0f)...\n', safe_x, safe_y);
                hw.moveToPose(safe_x, safe_y, HOVER_Z, C1_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
                pause(0.3);
            end

            fprintf('[C1.8] Rotate pitch %+.0f → %+.0f...\n', C1_PICK_PITCH, C1_PLACE_PITCH);
            hw.moveToPose(safe_x, safe_y, HOVER_Z, C1_PLACE_PITCH, MOVE_TIME*1.5, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            if needs_pullout
                fprintf('[C1.9] Return above pick XY...\n');
                hw.moveToPose(cpx1, cpy1, HOVER_Z, C1_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
                pause(0.3);
            end

            % Place sequence
            fprintf('[C1.10] Fly to target...\n');
            hw.moveToPose(ppx1, ppy1, HOVER_Z, C1_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C1.11] Approach z=%.1f...\n', ppz1 + PLACE_VERTICAL_OFFSET);
            hw.moveToPose(ppx1, ppy1, ppz1 + PLACE_VERTICAL_OFFSET, C1_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C1.12] Lower to place z=%.1f...\n', ppz1);
            hw.moveToPose(ppx1, ppy1, ppz1, C1_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            fprintf('[C1.13] Release...\n');
            hw.openGripper(); pause(1.0);

            fprintf('[C1.14] Retract...\n');
            hw.moveToPose(ppx1, ppy1, HOVER_Z, C1_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);
            goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            fprintf('=== Cube 1 placed (level 0) ===\n');

            % Tuning prompt
            cube1_done = true;
            if TUNING_MODE
                [cube1_done, C1_PICK_OFFSET, C1_PLACE_OFFSET] = ...
                    tuningPrompt('C1', C1_PICK_OFFSET, C1_PLACE_OFFSET);
            end
        end
    end

    % =====================================================================
    %  CUBE 2 — bridge pick
    %  Stack level 1 (on top of Cube 1)
    % =====================================================================
    if RUN_CUBE2
        cube2_done = false;
        while ~cube2_done
            fprintf('\n========== CUBE 2 — Bridge pick ==========\n');

            cpx2 = CUBE2_XY(1) + C2_PICK_OFFSET(1);
            cpy2 = CUBE2_XY(2) + C2_PICK_OFFSET(2);
            cpz2 = PICK_Z_BASE + C2_PICK_OFFSET(3);

            [ppx2, ppy2] = computePlaceXY(TARGET_XY(1), TARGET_XY(2), PLACE_OFFSET_MAG, C2_PLACE_OFFSET);
            ppz2 = PLACE_Z(1) + C2_PLACE_OFFSET(3);

            fprintf('  Pick:  (%.1f, %.1f, %.1f) pitch=%+.0f\n', cpx2, cpy2, cpz2, C2_PICK_PITCH);
            fprintf('  Place: (%.1f, %.1f, %.1f) pitch=%+.0f\n', ppx2, ppy2, ppz2, C2_PLACE_PITCH);

            pick_pose2       = [cpx2, cpy2, cpz2, C2_PICK_PITCH];
            lift_after_pick2 = [cpx2, cpy2, cpz2 + PICK_LIFT_MM, C2_PICK_PITCH];

            % Bridge-safe entry/exit waypoints
            approach_x2 = BRIDGE_X_MIN - X_RETREAT_OFFSET;
            exit_target2 = [approach_x2, cpy2, HOVER_Z, TRANSIT_PITCH];

            wp_entry2 = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
                HOME_POSE, pick_pose2, bridge_zone, bridge_zones, planner_opts);
            wp_exit2  = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
                lift_after_pick2, exit_target2, bridge_zone, bridge_zones, planner_opts);

            hw.openGripper(); pause(0.3);

            % Set gripper to bridge-safe width (50% open)
            fprintf('[C2.0] Bridge jaw width...\n');
            hw.setGripperPosition(50);
            pause(0.5);

            % Entry
            fprintf('[C2.1] Bridge entry (%d waypoints)...\n', size(wp_entry2,1));
            entry_ctx = struct('zones', bridge_zones, ...
                               'final_target_pose', wp_entry2(end,:), ...
                               'preplanned_route', true, ...
                               'dynamic_pitch', false);
            for i = 1:size(wp_entry2, 1)
                wp = wp_entry2(i,:);
                fprintf('  Entry %d/%d → [%.1f, %.1f, %.1f, %.1f]\n', ...
                    i, size(wp_entry2,1), wp(1), wp(2), wp(3), wp(4));
                em = MOTION_MODE;
                if i == size(wp_entry2, 1), em = 2; end  % task-space final slide
                hw.moveToPose(wp(1), wp(2), wp(3), wp(4), MOVE_TIME, em, Z_FLOOR, entry_ctx);
                pause(0.3);
            end

            fprintf('[C2.2] Final lock at pick...\n');
            hw.moveToPose(pick_pose2(1), pick_pose2(2), pick_pose2(3), pick_pose2(4), ...
                max(0.8, 0.5*MOVE_TIME), 1, Z_FLOOR, entry_ctx);
            pause(0.3);

            fprintf('[C2.3] Grip...\n');
            hw.closeGripper(); pause(PICK_HOLD_TIME);

            fprintf('[C2.4] Micro-lift +%d mm...\n', PICK_LIFT_MM);
            lift_ctx = struct('zones', bridge_zones, ...
                              'final_target_pose', lift_after_pick2, ...
                              'preplanned_route', true, ...
                              'dynamic_pitch', false);
            hw.moveToPose(lift_after_pick2(1), lift_after_pick2(2), lift_after_pick2(3), lift_after_pick2(4), ...
                MOVE_TIME, MOTION_MODE, Z_FLOOR, lift_ctx);
            pause(0.3);

            % Exit
            fprintf('[C2.5] Bridge exit (%d waypoints)...\n', size(wp_exit2,1));
            exit_ctx = struct('zones', bridge_zones, ...
                              'final_target_pose', wp_exit2(end,:), ...
                              'preplanned_route', true, ...
                              'dynamic_pitch', false);
            for i = 1:size(wp_exit2, 1)
                wp = wp_exit2(i,:);
                fprintf('  Exit %d/%d → [%.1f, %.1f, %.1f, %.1f]\n', ...
                    i, size(wp_exit2,1), wp(1), wp(2), wp(3), wp(4));
                hw.moveToPose(wp(1), wp(2), wp(3), wp(4), MOVE_TIME, MOTION_MODE, Z_FLOOR, exit_ctx);
                pause(0.3);
            end

            % Place on stack level 1
            fprintf('[C2.6] Fly to target...\n');
            hw.moveToPose(ppx2, ppy2, HOVER_Z, C2_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C2.7] Approach z=%.1f...\n', ppz2 + PLACE_VERTICAL_OFFSET);
            hw.moveToPose(ppx2, ppy2, ppz2 + PLACE_VERTICAL_OFFSET, C2_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C2.8] Lower z=%.1f...\n', ppz2);
            hw.moveToPose(ppx2, ppy2, ppz2, C2_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            fprintf('[C2.9] Release...\n');
            hw.openGripper(); pause(1.0);

            fprintf('[C2.10] Retract...\n');
            hw.moveToPose(ppx2, ppy2, HOVER_Z, C2_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);
            goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            fprintf('=== Cube 2 placed (level 1) ===\n');

            cube2_done = true;
            if TUNING_MODE
                [cube2_done, C2_PICK_OFFSET, C2_PLACE_OFFSET] = ...
                    tuningPrompt('C2', C2_PICK_OFFSET, C2_PLACE_OFFSET);
            end
        end
    end

    % =====================================================================
    %  CUBE 3 — standard pick
    %  Stack level 2 (on top of Cube 2)
    % =====================================================================
    if RUN_CUBE3
        cube3_done = false;
        while ~cube3_done
            fprintf('\n========== CUBE 3 — Standard pick ==========\n');

            cpx3 = CUBE3_XY(1) + C3_PICK_OFFSET(1);
            cpy3 = CUBE3_XY(2) + C3_PICK_OFFSET(2);
            cpz3 = PICK_Z_BASE + C3_PICK_OFFSET(3);

            [ppx3, ppy3] = computePlaceXY(TARGET_XY(1), TARGET_XY(2), PLACE_OFFSET_MAG, C3_PLACE_OFFSET);
            ppz3 = PLACE_Z(2) + C3_PLACE_OFFSET(3);

            fprintf('  Pick:  (%.1f, %.1f, %.1f) pitch=%+.0f\n', cpx3, cpy3, cpz3, C3_PICK_PITCH);
            fprintf('  Place: (%.1f, %.1f, %.1f) pitch=%+.0f\n', ppx3, ppy3, ppz3, C3_PLACE_PITCH);

            hw.openGripper(); pause(0.3);

            fprintf('[C3.1] Above cube...\n');
            hw.moveToPose(cpx3, cpy3, HOVER_Z, 0, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C3.2] Tilt pitch=%+.0f...\n', C3_PICK_PITCH);
            hw.moveToPose(cpx3, cpy3, HOVER_Z, C3_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C3.3] Descend z=%.1f...\n', cpz3);
            hw.moveToPose(cpx3, cpy3, cpz3, C3_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            fprintf('[C3.4] Grip...\n');
            hw.closeGripper(); pause(PICK_HOLD_TIME);

            fprintf('[C3.5] Micro-lift +%d mm...\n', PICK_LIFT_MM);
            hw.moveToPose(cpx3, cpy3, cpz3 + PICK_LIFT_MM, C3_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C3.6] Lift to hover...\n');
            hw.moveToPose(cpx3, cpy3, HOVER_Z, C3_PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            % Place on stack level 2
            fprintf('[C3.7] Fly to target...\n');
            hw.moveToPose(ppx3, ppy3, HOVER_Z, C3_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C3.8] Approach z=%.1f...\n', ppz3 + PLACE_VERTICAL_OFFSET);
            hw.moveToPose(ppx3, ppy3, ppz3 + PLACE_VERTICAL_OFFSET, C3_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);

            fprintf('[C3.9] Lower z=%.1f...\n', ppz3);
            hw.moveToPose(ppx3, ppy3, ppz3, C3_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.5);

            fprintf('[C3.10] Release...\n');
            hw.openGripper(); pause(1.0);

            fprintf('[C3.11] Retract...\n');
            hw.moveToPose(ppx3, ppy3, HOVER_Z, C3_PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);
            goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR);
            fprintf('=== Cube 3 placed (level 2 — top) ===\n');

            cube3_done = true;
            if TUNING_MODE
                [cube3_done, C3_PICK_OFFSET, C3_PLACE_OFFSET] = ...
                    tuningPrompt('C3', C3_PICK_OFFSET, C3_PLACE_OFFSET);
            end
        end
    end

    %% ── DONE ────────────────────────────────────────────────────────────
    fprintf('\n==========================================================\n');
    fprintf('  Task 2B COMPLETE — 3 cubes stacked at (%+.0f, %+.0f)\n', TARGET_XY);
    fprintf('  Final offsets:\n');
    fprintf('    C1 pick(%+.1f,%+.1f,%+.1f) place(%+.1f,%+.1f,%+.1f)\n', C1_PICK_OFFSET, C1_PLACE_OFFSET);
    fprintf('    C2 pick(%+.1f,%+.1f,%+.1f) place(%+.1f,%+.1f,%+.1f)\n', C2_PICK_OFFSET, C2_PLACE_OFFSET);
    fprintf('    C3 pick(%+.1f,%+.1f,%+.1f) place(%+.1f,%+.1f,%+.1f)\n', C3_PICK_OFFSET, C3_PLACE_OFFSET);
    fprintf('==========================================================\n');
    hw.disconnect();

catch ME
    fprintf('\nERROR: %s\n', ME.message);
    fprintf('Stack:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s  line %d\n', ME.stack(k).name, ME.stack(k).line);
    end
end


%% ── HELPERS (must be after all executable code) ─────────────────────────

function [px, py] = computePlaceXY(tx, ty, mag, extra)
%COMPUTEPLACEXY  Target XY with global offset toward origin + per-cube extra.
    r = sqrt(tx^2 + ty^2);
    if r > 1e-6
        px = tx - mag * tx / r + extra(1);
        py = ty - mag * ty / r + extra(2);
    else
        px = tx + extra(1);
        py = ty + extra(2);
    end
end

function goHome(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR)
    fprintf('[HOME] → home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);
end

function s = onoff(flag)
    if flag, s = 'ON'; else, s = 'OFF'; end
end

function [done, pick_off, place_off] = tuningPrompt(label, pick_off, place_off)
%TUNINGPROMPT  After a cube place, let the user adjust offsets and re-run.
    fprintf('\n--- %s Tuning ---\n', label);
    fprintf('  Current pick  offset: (%+.1f, %+.1f, %+.1f)\n', pick_off);
    fprintf('  Current place offset: (%+.1f, %+.1f, %+.1f)\n', place_off);
    fprintf('  Enter new offsets or press Enter to keep, ''n'' to accept & move on.\n');

    % Pick offset
    inp = input(sprintf('  %s pick  dX dY dZ [%+.1f %+.1f %+.1f]: ', label, pick_off), 's');
    if strcmpi(strtrim(inp), 'n')
        done = true; return;
    end
    if ~isempty(strtrim(inp))
        v = str2num(inp); %#ok<ST2NM>
        if numel(v) >= 3, pick_off = v(1:3);
        elseif numel(v) >= 2, pick_off(1:2) = v(1:2);
        end
    end

    % Place offset
    inp = input(sprintf('  %s place dX dY dZ [%+.1f %+.1f %+.1f]: ', label, place_off), 's');
    if strcmpi(strtrim(inp), 'n')
        done = true; return;
    end
    if ~isempty(strtrim(inp))
        v = str2num(inp); %#ok<ST2NM>
        if numel(v) >= 3, place_off = v(1:3);
        elseif numel(v) >= 2, place_off(1:2) = v(1:2);
        end
    end

    fprintf('  Updated:  pick(%+.1f,%+.1f,%+.1f)  place(%+.1f,%+.1f,%+.1f)\n', pick_off, place_off);
    fprintf('  Re-running %s with new offsets...\n\n', label);
    done = false;  % loop again
end

function safeShutdown(hw, HOME_POSE, MOVE_TIME, MOTION_MODE, Z_FLOOR)
    fprintf('\n>>> Emergency cleanup...\n');
    try hw.openGripper(); catch, end
    try
        hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
    catch, end
    try hw.disconnect(); catch, end
    fprintf('>>> Safe shutdown complete.\n');
end
