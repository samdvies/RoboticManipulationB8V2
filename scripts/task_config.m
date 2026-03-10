function cfg = task_config()
% TASK_CONFIG  Workspace configuration for the general task planner.
%
% Edit this file to change the workspace layout, cube positions, holder
% positions, and tuning parameters. The main script (task_general_planner.m)
% loads this on startup so you don't have to re-enter values each run.
%
% Usage:
%   cfg = task_config();

% =========================================================================
%  HARDWARE
% =========================================================================
cfg.PORT         = 'COM4';
cfg.BAUD         = 1000000;
cfg.VELOCITY     = 20;
cfg.MOVE_TIME    = 1.25;
cfg.MOTION_MODE  = 1;          % 1=joint interp, 2=task-space, 3=Jacobian
cfg.Z_FLOOR      = 15;
cfg.DRY_RUN      = false;      % true = IK/FK validation only, no hardware

% =========================================================================
%  CUBE CONSTANTS
% =========================================================================
cfg.CUBE_Z_SURFACE = 32.5;    % table surface to cube bottom (mm)
cfg.CUBE_SIZE      = 25;      % cube side length (mm)
cfg.HOVER_Z        = 32.5 + 80;  % 112.5 mm — safe travel height for pick
cfg.PLACE_APPROACH_Z = 180;   % mm — higher Z when moving to place (clear bridge)
cfg.PICK_Z_OFFSET_MM = 2;     % add to computed pick height (cube center), mm
cfg.PICK_LIFT_MM   = 5;       % small lift after gripping before full lift
cfg.PICK_HOLD_TIME = 2.5;     % pause after closing gripper (s)
cfg.HOME_POSE      = [134, 0, 240, -45];

% =========================================================================
%  PICK / PLACE OFFSETS
% =========================================================================
% Rotation pick offsets by holder angle (0°, 22.5°, 45°). Applied at PICK only;
% place location does not affect these. Tune [dx, dy] mm per angle for grip alignment.
cfg.PICK_OFFSET_0    = [5, 0];    % offset when picking from 0° holder (mm)
cfg.PICK_OFFSET_22_5 = [5, 0];    % offset when picking from 22.5° holder (tune)
cfg.PICK_OFFSET_45   = [5, 0];    % offset when picking from 45° holder (tune)
cfg.PLACE_OFFSET_MAG   = sqrt(32); % place offset magnitude toward origin (mm)
% Place offset by pick angle (0°, 22.5°, 45°). Used only when cube was rotated;
% links to the angle at which the cube was picked. Added to magnitude-based offset.
cfg.PLACE_OFFSET_0    = [0, 0];    % extra [dx, dy] when placing after rotation from 0° pick (tune)
cfg.PLACE_OFFSET_22_5 = [0, 0];   % extra [dx, dy] when placing after rotation from 22.5° pick (tune)
cfg.PLACE_OFFSET_45   = [4, 4];   % extra [dx, dy] when placing after rotation from 45° pick (tune)
cfg.PLACE_VERTICAL_OFFSET_MM = 25; % mm above place surface for "directly above" waypoint (straight-down approach)
cfg.PLACE_DROP_MM      = 1;       % release this many mm above rest so cube drops onto stack
cfg.MIN_ROTATE_RADIUS  = 200;      % min radius from base for safe pitch rotation (mm)

% =========================================================================
%  BRIDGE (always at Y~0, open toward robot i.e. lower X)
% =========================================================================
cfg.BRIDGE_POS             = [225, 0];   % bridge centre XY (mm)
cfg.BRIDGE_CLEARANCE_Z     = 85;         % Z clearance under bridge (mm)
cfg.BRIDGE_APPROACH_OFFSET = 60;         % approach from bridge_x - this (mm)

% =========================================================================
%  CUBE HOLDERS (all holder positions on the board, Nx2 [x, y])
%  List every holder — the planner tracks which are occupied.
%  holder_pickup_angles: 1 per holder, in degrees — 0, 22.5, or 45 (slot orientation).
% =========================================================================
cfg.holders = [
    150,  150;
    150, -150;
    100,  -175;
    225, 0;
];
cfg.holder_pickup_angles = [0; 0; 0; 0];  % 0, 22.5, or 45 for each holder (tune to match physical slots)

% =========================================================================
%  3 CUBES — entered in desired STACKING ORDER (bottom to top)
%  Each row: [x, y, under_bridge, needs_rotation]
%    under_bridge:  1 = yes, 0 = no
%    needs_rotation: 1 = yes, 0 = no
% =========================================================================
cfg.cubes = [
    225,    0,   1,   0;    % Cube 1 (bottom): under bridge, no rotation
    150, -150,   0,   1;    % Cube 2 (middle): needs rotation
    100,   -175,   0,   0;    % Cube 3 (top):    no rotation
];

% =========================================================================
%  TARGET — where to stack all 3 cubes
% =========================================================================
cfg.target = [150, 150];

% =========================================================================
%  PLACE XY OFFSET TUNING
%  Applied on top of the automatic offset model.
%  Adjust after test runs to fine-tune placement accuracy.
% =========================================================================
cfg.place_offset_tuning = [0, 0];

end
