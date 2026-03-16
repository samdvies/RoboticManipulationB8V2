function cfg = task_config()
% TASK_CONFIG  Workspace configuration for the general task planner (Task 2a).
%
% Edit this file to change the workspace layout, cube positions, holder
% positions, and tuning parameters. The main script (task_general_planner.m)
% loads this on startup so you don't have to re-enter values each run.
%
% Task 2a: Three cubes to three empty holders. Standard pick and place only.
% Per-cube placement offsets (radial, tangential) tune each cube's drop position.
%
% Usage:
%   cfg = task_config();

% =========================================================================
%  HARDWARE
% =========================================================================
cfg.PORT         = 'COM4';
cfg.BAUD         = 1000000;
cfg.VELOCITY     = 40;         % 2x speed (was 20)
cfg.MOVE_TIME    = 0.625;      % 2x speed: half time per move (was 1.25)
cfg.MOTION_MODE  = 1;          % 1=joint interp, 2=task-space, 3=Jacobian
cfg.Z_FLOOR      = 15;
cfg.DRY_RUN      = false;      % true = IK/FK validation only, no hardware

% =========================================================================
%  CUBE CONSTANTS
% =========================================================================
cfg.CUBE_Z_SURFACE = 32.5;    % table surface to cube bottom (mm)
cfg.CUBE_SIZE      = 25;      % cube side length (mm)
cfg.HOVER_Z        = 32.5 + 80;  % 112.5 mm — safe travel height for pick
cfg.PLACE_APPROACH_Z = 180;   % mm — travel/approach Z; reachable at TRANSIT_PITCH
cfg.TRANSIT_PITCH    = -45;   % pitch for hover/approach/retract (reachable at high Z)
cfg.PICK_Z_OFFSET_MM = 2;     % add to computed pick height (cube center), mm
cfg.PICK_LIFT_MM   = 5;       % small lift after gripping before full lift
cfg.PICK_HOLD_TIME = 1.25;     % pause after closing gripper (s)
cfg.HOME_POSE      = [134, 0, 240, -45];

% =========================================================================
%  PICK OFFSETS (standard pick only for Task 2a)
% =========================================================================
% When cfg.cube_pick_offsets exists, it overrides angle-based: row i = [radial, tangential]
% for cube i. Resolved at cube bearing atan2(y, x). Use [0,0] for a cube to pick at nominal pos.
cfg.PICK_OFFSET_STD_0    = [0, 0];  % fallback when pick angle ~0°
cfg.PICK_OFFSET_STD_22_5 = [0, 0];  % fallback when pick angle ~22.5°
cfg.PICK_OFFSET_STD_45   = [0, 0];  % fallback when pick angle ~45°

% Per-cube pick offset [radial_mm, tangential_mm] — row i = cube picked in position i.
cfg.cube_pick_offsets = [
    0,  0;   % picked 1st
    0,  0;   % picked 2nd
    0,  0;   % picked 3rd
];

% =========================================================================
%  PLACE (common)
% =========================================================================
cfg.PLACE_VERTICAL_OFFSET_MM = 25; % mm above place surface for "directly above" waypoint
cfg.PLACE_DROP_MM           = 1;  % release this many mm above rest

% =========================================================================
%  CUBE HOLDERS (all holder positions on the board, Nx2 [x, y])
% =========================================================================
cfg.holders = [
    150,  150;
    100,    0;
    0, 150;
    75, -200;
    175, -175;
    225,    0;
];

% =========================================================================
%  TASK 2a: 3 CUBES — pos [x,y] only (no bridge, no rotation)
%  Pick order = row order. First row picked first, etc.
% =========================================================================
cfg.cubes = [
    75, -200;       % picked 1st
    175, -175;      % picked 2nd
    225,    0;      % picked 3rd
];

% =========================================================================
%  PLACE TARGETS (optional; planner auto-assigns cubes to initially empty holders)
%  The planner finds holders with no cube at start and assigns cube 1 -> 1st empty,
%  cube 2 -> 2nd empty, cube 3 -> 3rd empty. So each place is to an empty holder.
% =========================================================================
cfg.cube_place_targets = cfg.holders(1:3, :);  % unused for assignment; kept for reference

% =========================================================================
%  PER-CUBE PLACE OFFSET [radial_mm, tangential_mm]
%  Resolved at place target bearing (radial > 0 = away from base; tangent > 0 = CCW).
%  CUBE 1: pos (above), offset (below). CUBE 2, CUBE 3 same.
% =========================================================================
cfg.cube_place_offsets = [
    -3,  7;   % picked 1st
    -6,  0;   % picked 2nd
     1,  5;   % picked 3rd
];

% Per-cube release height adjustment (mm). Negative = place lower.
cfg.cube_release_z_adjust = [
     0;   % picked 1st
   -10;   % picked 2nd
     0;   % picked 3rd
];

% =========================================================================
%  GLOBAL PLACE OFFSET TUNING (added on top of per-cube offsets)
% =========================================================================
cfg.place_offset_tuning = [0, 0];

end
