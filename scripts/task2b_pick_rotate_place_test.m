% TASK2B_PICK_ROTATE_PLACE_TEST  Pick & rotate with hardcoded offset model for place.
%
% Usage:
%   run('scripts/task2b_pick_rotate_place_test.m')
%
% Purpose:
%   Test whether place offset generalizes: offset = magnitude toward origin.
%   Hardcoded: pick X offset = 5 mm; place offset magnitude = sqrt(32) mm
%   in direction from (PLACE_X, PLACE_Y) toward origin.
%
%   At run time you're prompted for place X Y (mm); press Enter for default 175 175.

clc; clear;
addpath(genpath('../src'));

% =========================================================================
%  HARDWARE CONFIGURATION
% =========================================================================
PORT      = 'COM4';
BAUD      = 1000000;
VELOCITY  = 20;
MOVE_TIME = 1.25;
PICK_HOLD_TIME = 2.5;
PICK_LIFT_MM   = 5;
Z_FLOOR   = 15;
MOTION_MODE = 1;

% =========================================================================
%  HARDCODED OFFSETS (no tuning loop)
% =========================================================================
PICK_OFFSET_X  = 0;              % pick X offset (mm), 0 = nominal pick position
PLACE_OFFSET_MAGNITUDE = sqrt(32); % place offset magnitude (mm); direction = toward origin from place

% =========================================================================
%  CUBE / TASK (PLACE_X, PLACE_Y set by prompt below; defaults here)
% =========================================================================
PICK_X         = 200;
PICK_Y         = 0;
PLACE_X_DEFAULT = 175;
PLACE_Y_DEFAULT = 175;
CUBE_Z_SURFACE = 32.5;
CUBE_SIZE      = 25;
ROTATE_MODE    = 1;

HOME_POSE = [134, 0, 240, -45];
MIN_ROTATE_RADIUS = 200;

try
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(0.5);

    % ── Prompt for place position (inline) ────────────────────────────────
    inp = input(sprintf('Enter place position X Y (mm) [Enter = %.0f %.0f]: ', PLACE_X_DEFAULT, PLACE_Y_DEFAULT), 's');
    if isempty(strtrim(inp))
        PLACE_X = PLACE_X_DEFAULT;
        PLACE_Y = PLACE_Y_DEFAULT;
    else
        v = str2num(inp); %#ok<ST2NM>
        if numel(v) >= 2
            PLACE_X = v(1);
            PLACE_Y = v(2);
        else
            PLACE_X = PLACE_X_DEFAULT;
            PLACE_Y = PLACE_Y_DEFAULT;
            fprintf('Using default place (%.0f, %.0f).\n', PLACE_X, PLACE_Y);
        end
    end
    fprintf('Place target: (%.1f, %.1f) mm\n', PLACE_X, PLACE_Y);

    % ── Derived: pick position ───────────────────────────────────────────
    pick_x = PICK_X + PICK_OFFSET_X;
    pick_y = PICK_Y;
    pick_z = CUBE_Z_SURFACE + CUBE_SIZE/2;
    hover_z = CUBE_Z_SURFACE + 80;

    % ── Derived: place offset from magnitude + angle (toward origin) ─────
    r_place = sqrt(PLACE_X^2 + PLACE_Y^2);
    if r_place > 1e-6
        place_offset_x = -PLACE_OFFSET_MAGNITUDE * PLACE_X / r_place;
        place_offset_y = -PLACE_OFFSET_MAGNITUDE * PLACE_Y / r_place;
    else
        place_offset_x = 0;
        place_offset_y = 0;
    end
    place_x = PLACE_X + place_offset_x;
    place_y = PLACE_Y + place_offset_y;
    place_z = pick_z;

    fprintf('Place nominal: (%.1f, %.1f)  -> offset (%.2f, %.2f)  -> actual (%.1f, %.1f)\n', ...
        PLACE_X, PLACE_Y, place_offset_x, place_offset_y, place_x, place_y);

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

    % ── Home ─────────────────────────────────────────────────────────────
    fprintf('[1] Home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    fprintf('[2] Above cube...\n');
    hw.moveToPose(pick_x, pick_y, hover_z, 0, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);

    fprintf('[3] Vertical grip...\n');
    hw.moveToPose(pick_x, pick_y, hover_z, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);

    fprintf('[4] Descend to cube...\n');
    hw.moveToPose(pick_x, pick_y, pick_z, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    fprintf('[5] Grip...\n');
    hw.closeGripper();
    pause(PICK_HOLD_TIME);

    fprintf('[5b] Straight up %d mm...\n', PICK_LIFT_MM);
    hw.moveToPose(pick_x, pick_y, pick_z + PICK_LIFT_MM, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);

    fprintf('[6] Lift...\n');
    hw.moveToPose(pick_x, pick_y, hover_z, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    if needs_pullout
        fprintf('[6b] Safe radius...\n');
        hw.moveToPose(safe_x, safe_y, hover_z, PICK_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);
    end

    if DO_ROTATE
        fprintf('[7] Rotate pitch...\n');
        rot_x = pick_x; rot_y = pick_y;
        if needs_pullout, rot_x = safe_x; rot_y = safe_y; end
        hw.moveToPose(rot_x, rot_y, hover_z, PLACE_PITCH, MOVE_TIME * 1.5, MOTION_MODE, Z_FLOOR);
        pause(0.5);
    end

    if needs_pullout
        fprintf('[7b] Return above cube...\n');
        hw.moveToPose(pick_x, pick_y, hover_z, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);
    end

    fprintf('[8] Above place...\n');
    hw.moveToPose(place_x, place_y, hover_z, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);

    fprintf('[9] Lower to place...\n');
    hw.moveToPose(place_x, place_y, place_z, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    fprintf('[10] Release...\n');
    hw.openGripper();
    pause(1.0);

    fprintf('[11] Retract & home...\n');
    hw.moveToPose(place_x, place_y, hover_z, PLACE_PITCH, MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    fprintf('\n=== Done. Run again and enter place X Y at the prompt to test another position. ===\n');
    hw.disconnect();

catch ME
    fprintf('\nERROR: %s\n', ME.message);
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
        end
    end
end
