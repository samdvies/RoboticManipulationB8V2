% TASK2B_PICK_ROTATE  Pick a cube and rotate it in place (pitch -90 -> -90/0/+90)
%
% Usage:
%   run('scripts/task2b_pick_rotate.m')
%
% Description:
%   1. Move above cube (pitch = 0)
%   2. Tilt pitch to -90 (vertical grip)
%   3. Descend to cube centre
%   4. Close gripper
%   5. Lift cube
%   6. Rotate pitch -90 -> 0  (in place, same x/y/z)
%   7. Lower cube to original spot (pitch = 0)
%   8. Open gripper / release
%   9. Retract and return home
%
% Calibration:
%   With TUNING_LOOP = true: first run uses no offsets. When the arm reaches
%   the drop position (above place at hover Z), you enter X,Y offsets for the
%   *next* run. Script then places, retracts, and loops: place cube at pick,
%   press Enter to run again with the new offsets. Repeat until offsets look
%   good, then enter 'n' to finish.

clc; clear;
addpath(genpath('../src'));

% =========================================================================
%  HARDWARE CONFIGURATION
% =========================================================================
PORT      = 'COM4';
BAUD      = 1000000;
VELOCITY  = 20;          % joint velocity (lower = slower, safer)
MOVE_TIME = 1.25;        % seconds per waypoint move (2x speed)
PICK_HOLD_TIME = 2.5;    % seconds to hold after closing gripper before lifting (increase if moving up too fast)
PICK_LIFT_MM   = 5;      % straight up (mm) after pick before other waypoints
Z_FLOOR   = 15;          % safety floor – arm won't go below this Z (mm)
MOTION_MODE = 1;         % 1=Joint Interp, 2=Task Linear, 3=Jacobian Hybrid
TUNING_LOOP = true;      % true = pause at drop position, prompt for X/Y offsets, re-run until done

% =========================================================================
%  CALIBRATION OFFSETS  –– start at 0; updated by tuning loop when TUNING_LOOP
%  All units: mm for X/Y/Z
% =========================================================================
OFFSET_X_BASE =  0;   % positive = further forward
OFFSET_Y_BASE =  0;   % positive = further left
OFFSET_Z_BASE = 0;   % positive = higher (pick height = CUBE_Z_SURFACE + CUBE_SIZE/2 = 32.5 + 12.5 = 45 mm)

OFFSET_X_POS90 = 0;
OFFSET_Y_POS90 = 0;
OFFSET_Z_POS90 = 0;

OFFSET_X_NEG90 = 0;
OFFSET_Y_NEG90 = 0;
OFFSET_Z_NEG90 = 0;

% =========================================================================
%  CUBE / TASK PARAMETERS
% =========================================================================
PICK_X         = 200;   % pick centre X  (mm, user frame)
PICK_Y         =   0;   % pick centre Y  (mm, user frame)
PICK_OFFSET_X  = 5;     % offset applied to pick X only (e.g. 5 = further forward for easier drop)
PLACE_X        = 175;   % place centre X (mm, user frame)
PLACE_Y        = 175;   % place centre Y (mm, user frame)
CUBE_Z_SURFACE = 32.5; % Z of table surface (mm); pick/place height = 32.5 + 12.5 = 45 mm
CUBE_SIZE      =  25;   % cube side length (mm)

ROTATE_MODE    = 1;     % 0=no, +1=+90, -1=-90

HOME_POSE = [134, 0, 240, -45];
MIN_ROTATE_RADIUS = 200;

try
    % ── Connect ──────────────────────────────────────────────────────────
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(0.5);

    % Optional: set initial place offsets so you don't have to tune from zero each time
    inp = input('Enter initial place offsets X Y (mm), or press Enter for 0 0: ', 's');
    if ~isempty(strtrim(inp))
        v = str2num(inp); %#ok<ST2NM>
        if numel(v) >= 2
            OFFSET_X_BASE = v(1);
            OFFSET_Y_BASE = v(2);
            fprintf('Using initial place offsets X=%+.1f  Y=%+.1f\n', OFFSET_X_BASE, OFFSET_Y_BASE);
        end
    end

    run_again = true;
    while run_again
        % ── Recompute derived positions (uses current OFFSET_*_BASE) ───────
        OFFSET_X = OFFSET_X_BASE;
        OFFSET_Y = OFFSET_Y_BASE;
        OFFSET_Z = OFFSET_Z_BASE;
        if ROTATE_MODE > 0
            OFFSET_X = OFFSET_X + OFFSET_X_POS90;
            OFFSET_Y = OFFSET_Y + OFFSET_Y_POS90;
            OFFSET_Z = OFFSET_Z + OFFSET_Z_POS90;
        elseif ROTATE_MODE < 0
            OFFSET_X = OFFSET_X + OFFSET_X_NEG90;
            OFFSET_Y = OFFSET_Y + OFFSET_Y_NEG90;
            OFFSET_Z = OFFSET_Z + OFFSET_Z_NEG90;
        end

        % Pick: nominal position + optional pick offset (place uses tuning offsets only)
        pick_x  = PICK_X + PICK_OFFSET_X;
        pick_y  = PICK_Y;
        pick_z  = CUBE_Z_SURFACE + CUBE_SIZE/2 + OFFSET_Z;
        place_x = PLACE_X + OFFSET_X;
        place_y = PLACE_Y + OFFSET_Y;
        place_z = pick_z;
        hover_z = CUBE_Z_SURFACE + 80;

        if ROTATE_MODE < 0
            PICK_PITCH = 0;
        else
            PICK_PITCH = -90;
        end
        if ROTATE_MODE > 0
            PLACE_PITCH = 0;
        elseif ROTATE_MODE < 0
            PLACE_PITCH = -90;
        else
            PLACE_PITCH = PICK_PITCH;
        end
        DO_ROTATE = (ROTATE_MODE ~= 0);

        r_cube = sqrt(pick_x^2 + pick_y^2);
        if r_cube < MIN_ROTATE_RADIUS && r_cube > 0.001
            scale = MIN_ROTATE_RADIUS / r_cube;
            safe_x = pick_x * scale;
            safe_y = pick_y * scale;
        else
            safe_x = pick_x;
            safe_y = pick_y;
        end
        needs_pullout = (r_cube < MIN_ROTATE_RADIUS);

        fprintf('\n=== Task 2b run (offsets X=%+.1f Y=%+.1f) ===\n', OFFSET_X, OFFSET_Y);

        % ── Home ─────────────────────────────────────────────────────────
        fprintf('[1] Moving to home...\n');
        hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.5);

        % ── Phase 1: Above cube ──────────────────────────────────────────
        fprintf('[2] Moving above cube (pitch=0)...\n');
        hw.moveToPose(pick_x, pick_y, hover_z, 0, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);

        % ── Phase 2: Tilt to vertical grip ───────────────────────────────
        fprintf('[3] Tilting to vertical grip...\n');
        hw.moveToPose(pick_x, pick_y, hover_z, PICK_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);

        % ── Phase 3: Descend to cube ──────────────────────────────────────
        fprintf('[4] Descending to cube...\n');
        hw.moveToPose(pick_x, pick_y, pick_z, PICK_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.5);

        % ── Phase 4: Close gripper ────────────────────────────────────────
        fprintf('[5] Gripping cube...\n');
        hw.closeGripper();
        pause(PICK_HOLD_TIME);

        % ── Phase 4b: Straight up 5 mm ────────────────────────────────────
        fprintf('[5b] Lifting straight up %d mm...\n', PICK_LIFT_MM);
        hw.moveToPose(pick_x, pick_y, pick_z + PICK_LIFT_MM, PICK_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);

        % ── Phase 5: Lift ─────────────────────────────────────────────────
        fprintf('[6] Lifting cube...\n');
        hw.moveToPose(pick_x, pick_y, hover_z, PICK_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.5);

        % ── Phase 5b: Pull out to safe radius if needed ────────────────────
        if needs_pullout
            fprintf('[6b] Moving to safe radius...\n');
            hw.moveToPose(safe_x, safe_y, hover_z, PICK_PITCH, ...
                          MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);
        end

        % ── Phase 6: Rotate pitch ────────────────────────────────────────
        rot_x = pick_x; rot_y = pick_y;
        if needs_pullout, rot_x = safe_x; rot_y = safe_y; end
        if DO_ROTATE
            fprintf('[7] Rotating pitch...\n');
            hw.moveToPose(rot_x, rot_y, hover_z, PLACE_PITCH, ...
                          MOVE_TIME * 1.5, MOTION_MODE, Z_FLOOR);
            pause(0.5);
        end

        % ── Phase 6b: Return above pick if we pulled out ───────────────────
        if needs_pullout
            fprintf('[7b] Returning above cube...\n');
            hw.moveToPose(pick_x, pick_y, hover_z, PLACE_PITCH, ...
                          MOVE_TIME, MOTION_MODE, Z_FLOOR);
            pause(0.3);
        end

        % ── Phase 6c: Move above place (hover) – drop position for tuning ───
        fprintf('[8] Moving above place (hover)...\n');
        hw.moveToPose(place_x, place_y, hover_z, PLACE_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);

        % ── Tuning: prompt for X,Y offsets for next run ───────────────────
        if TUNING_LOOP
            fprintf('\n--- Offset tuning (arm at drop position, high Z) ---\n');
            fprintf('Enter X,Y offset (mm) for NEXT run. Positive X=forward, Y=left.\n');
            inp_x = input(sprintf('  X offset (mm) [current %+.1f]; or ''n'' to finish: ', OFFSET_X_BASE), 's');
            if strcmpi(strtrim(inp_x), 'n')
                run_again = false;
            else
                dx = str2double(inp_x);
                if isnan(dx), dx = OFFSET_X_BASE; end
                inp_y = input(sprintf('  Y offset (mm) [current %+.1f]: ', OFFSET_Y_BASE), 's');
                if strcmpi(strtrim(inp_y), 'n')
                    run_again = false;
                else
                    dy = str2double(inp_y);
                    if isnan(dy), dy = OFFSET_Y_BASE; end
                    OFFSET_X_BASE = dx;
                    OFFSET_Y_BASE = dy;
                    fprintf('  Next run will use offsets X=%+.1f  Y=%+.1f\n', OFFSET_X_BASE, OFFSET_Y_BASE);
                end
            end
        else
            run_again = false;
        end

        % ── Phase 7: Lower to place ───────────────────────────────────────
        fprintf('[9] Lowering cube at place...\n');
        hw.moveToPose(place_x, place_y, place_z, PLACE_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.5);

        % ── Phase 8: Open gripper ─────────────────────────────────────────
        fprintf('[10] Releasing cube...\n');
        hw.openGripper();
        pause(1.0);

        % ── Phase 9: Retract and home ─────────────────────────────────────
        fprintf('[11] Retracting...\n');
        hw.moveToPose(place_x, place_y, hover_z, PLACE_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);
        fprintf('[12] Returning home...\n');
        hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.5);

        % ── Loop again? ───────────────────────────────────────────────────
        if TUNING_LOOP && run_again
            fprintf('\nPlace cube at pick (%.0f, %.0f) mm, then press Enter to run again (or ''n'' to quit).\n', PICK_X, PICK_Y);
            r = input('', 's');
            if strcmpi(strtrim(r), 'n')
                run_again = false;
            end
        else
            run_again = false;
        end
    end

    fprintf('\n=== Task 2b complete (final offsets X=%+.1f Y=%+.1f) ===\n', OFFSET_X_BASE, OFFSET_Y_BASE);
    hw.disconnect();

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
