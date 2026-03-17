function cfg = task_config()
% TASK_CONFIG Workspace configuration for the general task planner (Task 2a).
%
% Task 2a moves three cubes from their starting holders to the three
% initially empty holders.
%
% Cube rows are [x, y, is_hard]:
% - is_hard = 0 -> standard pitch -90 pick/place
% - is_hard = 1 -> pitch 0 pick/place
%
% Per-cube pick/place offsets stay in this file so each cube can be tuned
% independently without touching motion code.

% =========================================================================
% HARDWARE
% =========================================================================
cfg.PORT = 'COM4';
cfg.BAUD = 1000000;
cfg.VELOCITY = 40;          % 2x speed (was 20)
cfg.MOVE_TIME = 0.625;      % 2x speed: half time per move (was 1.25)
cfg.MOTION_MODE = 1;        % 1=joint interp, 2=task-space, 3=Jacobian
cfg.Z_FLOOR = 15;
cfg.DRY_RUN = false;        % true = IK/FK validation only, no hardware

% =========================================================================
% CUBE CONSTANTS
% =========================================================================
cfg.CUBE_Z_SURFACE = 32.5;      % table surface to cube bottom (mm)
cfg.CUBE_SIZE = 25;             % cube side length (mm)
cfg.HOVER_Z = 32.5 + 80;        % 112.5 mm - standard pick hover height
cfg.PLACE_APPROACH_Z = 180;     % travel/approach Z used before place
cfg.TRANSIT_PITCH = -45;        % standard transit pitch
cfg.HARD_TRANSIT_PITCH = 0;     % hard cube transit pitch
cfg.HARD_APPROACH_Z = 180;      % hard cube pick approach / retract height
cfg.PICK_Z_OFFSET_MM = 2;       % added to computed cube-center pick height
cfg.PICK_LIFT_MM = 5;           % small lift after gripping before full lift
cfg.PICK_HOLD_TIME = 1.25;      % pause after closing gripper (s)
cfg.HOME_POSE = [134, 0, 240, -45];

% =========================================================================
% PICK OFFSETS
% =========================================================================
% Fallback angle-based standard offsets. Per-cube offsets below override
% these when cube_index is supplied.
cfg.PICK_OFFSET_STD_0 = [0, 0];
cfg.PICK_OFFSET_STD_22_5 = [0, 0];
cfg.PICK_OFFSET_STD_45 = [0, 0];

% Per-cube pick offset [radial_mm, tangential_mm].
% Tune the row for the cube's configured behavior (soft or hard).
cfg.cube_pick_offsets = [
    0, 0;    % Cube 1
    0, 0;    % Cube 2
    0, 0;    % Cube 3
];

% =========================================================================
% PLACE
% =========================================================================
cfg.PLACE_VERTICAL_OFFSET_MM = 25;   % directly above place surface
cfg.PLACE_DROP_MM = 1;               % release slightly above the surface

% =========================================================================
% HOLDERS
% =========================================================================
cfg.holders = [
    100,  100;   % H1 - Cube 1 start
    100, -200;   % H2 - Cube 2 start
      0, -100;   % H3 - Cube 3 start
      0,  200;   % H4 - Cube 1 dest
    175,  175;   % H5 - Cube 2 dest
    175,    0;   % H6 - Cube 3 dest
];

% =========================================================================
% TASK 2a CUBES
% =========================================================================
% [x, y, is_hard]
cfg.cubes = [
    100,  100, 0;   % Cube 1 - standard
    100, -200, 1;   % Cube 2 - hard
      0, -100, 0;   % Cube 3 - standard
];

% Forced execution order and explicit placement targets.
cfg.cube_forced_order = [1, 2, 3];
cfg.cube_forced_targets = [
      0,  200;   % Cube 1 -> (0, 200)
    175,  175;   % Cube 2 -> (175, 175)
    175,    0;   % Cube 3 -> (175, 0)
];

% Kept for reference only. Holder assignment is now planner-driven.
cfg.cube_place_targets = cfg.holders(1:3, :);

% Per-cube place offset [radial_mm, tangential_mm].
cfg.cube_place_offsets = [
    -3,  7;   % Cube 1
    -6,  0;   % Cube 2
     1,  5;   % Cube 3
];

% Per-cube release height adjustment (mm). Negative = place lower.
cfg.cube_release_z_adjust = [
     0;   % Cube 1
   -10;   % Cube 2
     0;   % Cube 3
];

% Global place offset tuning added on top of the per-cube place offset.
cfg.place_offset_tuning = [0, 0];

end
