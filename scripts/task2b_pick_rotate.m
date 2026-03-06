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
%   Adjust the OFFSET_ constants below after each trial run until the
%   gripper hits the target position accurately.

clc; clear;
addpath(genpath('../src'));

% =========================================================================
%  HARDWARE CONFIGURATION
% =========================================================================
PORT      = 'COM4';
BAUD      = 1000000;
VELOCITY  = 20;          % joint velocity (lower = slower, safer)
MOVE_TIME = 2.5;         % seconds per waypoint move
Z_FLOOR   = 15;          % safety floor – arm won't go below this Z (mm)
MOTION_MODE = 1;         % 1=Joint Interp, 2=Task Linear, 3=Jacobian Hybrid

% =========================================================================
%  CALIBRATION OFFSETS  –– tune these by trial and error
%  All units: mm for X/Y/Z
% =========================================================================
% Base offsets (common to all modes)
OFFSET_X_BASE =  0;   % positive = further forward
OFFSET_Y_BASE =  0;   % positive = further left
OFFSET_Z_BASE =  0;   % positive = higher (raise pick height)

% Additional offsets for +90° rotation
OFFSET_X_POS90 = 0;
OFFSET_Y_POS90 = 0;
OFFSET_Z_POS90 = 0;

% Additional offsets for -90° rotation
OFFSET_X_NEG90 = 0;
OFFSET_Y_NEG90 = 0;
OFFSET_Z_NEG90 = 0;

% =========================================================================
%  CUBE / TASK PARAMETERS
% =========================================================================
% Pick location (where cube starts)
PICK_X         = 200;   % pick centre X  (mm, user frame)
PICK_Y         =   0;   % pick centre Y  (mm, user frame)

% Place location (where cube should be set down)
PLACE_X        = 250;   % place centre X (mm, user frame)
PLACE_Y        =  50;   % place centre Y (mm, user frame)

CUBE_Z_SURFACE =  45;   % Z of table surface where cube bottom rests (mm)
CUBE_SIZE      =  25;   % cube side length (mm)

% Rotation mode:
%   0  -> no rotation
%   +1 -> rotate -90 -> 0 degrees
%   -1 -> rotate 0   -> -90 degrees
ROTATE_MODE    = 1;

% =========================================================================
%  DERIVED POSITIONS  (apply offsets here)
% =========================================================================
% Select offsets based on rotation mode
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

pick_x  = PICK_X  + OFFSET_X;
pick_y  = PICK_Y  + OFFSET_Y;
pick_z  = CUBE_Z_SURFACE + CUBE_SIZE/2 + OFFSET_Z;   % grip at cube centre

place_x = PLACE_X + OFFSET_X;
place_y = PLACE_Y + OFFSET_Y;
place_z = pick_z;                                    % same table height

hover_z = CUBE_Z_SURFACE + 80;                       % safe hover height

% Pick and place pitch based on rotation mode:
%   ROTATE_MODE > 0: pick -90, place 0
%   ROTATE_MODE < 0: pick 0,   place -90
%   ROTATE_MODE = 0: pick and place both -90 (no rotation)
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

HOME_POSE = [134, 0, 240, -45];

% Safe radius: if cube is too close to base for pitch=0, pull out first.
% Tool = 126mm, wrist needs ≥ ~60mm radially → min ≈ 200mm.
MIN_ROTATE_RADIUS = 200;
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

fprintf('=== Task 2b: Pick & Rotate ===\n');
fprintf('Pick target : (%.1f, %.1f)  surface Z=%.1f\n', ...
    PICK_X, PICK_Y, CUBE_Z_SURFACE);
fprintf('Place target: (%.1f, %.1f)\n', ...
    PLACE_X, PLACE_Y);
fprintf('Pick pose   : (%.1f, %.1f, %.1f)  pitch=%.1f\n', ...
    pick_x, pick_y, pick_z, PICK_PITCH);
fprintf('Active offsets: X=%+.1f  Y=%+.1f  Z=%+.1f\n', ...
    OFFSET_X, OFFSET_Y, OFFSET_Z);
fprintf('Rotate mode : %d (0=no, +1=+90, -1=-90)\n', ROTATE_MODE);
if needs_pullout
    fprintf('Safe radius pull-out to (%.1f, %.1f) for rotation.\n', safe_x, safe_y);
end
fprintf('\n');

try
    % ── Connect ──────────────────────────────────────────────────────────
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    hw.openGripper();
    pause(0.5);

    % ── Home ─────────────────────────────────────────────────────────────
    fprintf('[1/9] Moving to home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    % ── Phase 1: Move above cube (pitch = 0) ─────────────────────────────
    fprintf('[2/9] Moving above cube (pitch=0)...\n');
    hw.moveToPose(pick_x, pick_y, hover_z, 0, ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);

    % ── Phase 2: Tilt pitch to -90 in place ──────────────────────────────
    fprintf('[3/9] Tilting to vertical grip (pitch=%g)...\n', PICK_PITCH);
    hw.moveToPose(pick_x, pick_y, hover_z, PICK_PITCH, ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);

    % ── Phase 3: Descend to cube ──────────────────────────────────────────
    fprintf('[4/9] Descending to cube (z=%.1f)...\n', pick_z);
    hw.moveToPose(pick_x, pick_y, pick_z, PICK_PITCH, ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    % ── Phase 4: Close gripper ────────────────────────────────────────────
    fprintf('[5/9] Gripping cube...\n');
    hw.closeGripper();
    pause(1.5);   % hold for a moment to let gripper settle

    % ── Phase 5: Lift ────────────────────────────────────────────────────
    fprintf('[6/9] Lifting cube...\n');
    hw.moveToPose(pick_x, pick_y, hover_z, PICK_PITCH, ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    % ── Phase 5b: Pull out to safe radius if needed ────────────────────────
    if needs_pullout
        fprintf('[6b] Moving to safe radius (%.1f, %.1f)...\n', safe_x, safe_y);
        hw.moveToPose(safe_x, safe_y, hover_z, PICK_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);
    end

    % ── Phase 6: Rotate pitch -90 -> 0 (optional) ────────────────────────
    rot_x = pick_x; rot_y = pick_y;
    if needs_pullout, rot_x = safe_x; rot_y = safe_y; end
    if DO_ROTATE
        fprintf('[7/9] Rotating pitch %g -> %g...\n', PICK_PITCH, PLACE_PITCH);
        hw.moveToPose(rot_x, rot_y, hover_z, PLACE_PITCH, ...
                      MOVE_TIME * 1.5, MOTION_MODE, Z_FLOOR);   % slower for rotation
        pause(0.5);
    else
        fprintf('[7/9] Skipping rotation (DO_ROTATE=false)...\n');
    end

    % ── Phase 6b: Return to cube x/y if we pulled out ─────────────────────
    if needs_pullout
        fprintf('[7b] Returning above cube...\n');
        hw.moveToPose(pick_x, pick_y, hover_z, PLACE_PITCH, ...
                      MOVE_TIME, MOTION_MODE, Z_FLOOR);
        pause(0.3);
    end

    % ── Phase 7: Lower to place ───────────────────────────────────────────
    fprintf('[8/9] Lowering cube at place (pitch=%.1f)...\n', PLACE_PITCH);
    hw.moveToPose(place_x, place_y, place_z, PLACE_PITCH, ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    % ── Phase 8: Open gripper (release) ──────────────────────────────────
    fprintf('[9/9] Releasing cube...\n');
    hw.openGripper();
    pause(1.0);

    % ── Phase 9: Retract and return home ─────────────────────────────────
    fprintf('[+]  Retracting from place...\n');
    hw.moveToPose(place_x, place_y, hover_z, PLACE_PITCH, ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.3);

    fprintf('[+]  Returning home...\n');
    hw.moveToPose(HOME_POSE(1), HOME_POSE(2), HOME_POSE(3), HOME_POSE(4), ...
                  MOVE_TIME, MOTION_MODE, Z_FLOOR);
    pause(0.5);

    fprintf('\n=== Task 2b complete! ===\n');
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
