%% task_general_planner.m
% General Task Planner — Task 2a: Three cubes to three empty holders.
%
% Standard pick and place only (no bridge, no rotate). Loads workspace from
% task_config.m, plans pick→place per cube, validates via IK/FK, then executes.
%
% Usage (from project root):
%   run('scripts/task_general_planner.m')
%
% To change the workspace layout, edit scripts/task_config.m.
% Set cfg.DRY_RUN = true in the config to validate without hardware.

clc; clear;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
addpath(genpath(fullfile(script_dir, '..', 'src')));
addpath(genpath(fullfile(script_dir, '..', 'Common')));

%% ======================== LOAD CONFIG ========================

cfg = task_config();

%% ======================== EMPTY HOLDERS (place targets) ========================
% Each cube is placed on an initially EMPTY holder (no cube starts there).
holder_occupied = false(size(cfg.holders, 1), 1);
for h = 1:size(cfg.holders, 1)
    for c = 1:size(cfg.cubes, 1)
        if norm(cfg.cubes(c, 1:2) - cfg.holders(h, :)) < 5
            holder_occupied(h) = true;
            break;
        end
    end
end
empty_holder_idx = find(~holder_occupied);
n_cubes = size(cfg.cubes, 1);
if length(empty_holder_idx) < n_cubes
    error('Not enough empty holders: %d cubes but only %d empty holders. Ensure cube positions and holder list leave at least %d holders without a cube.', ...
        n_cubes, length(empty_holder_idx), n_cubes);
end
empty_holder_idx = empty_holder_idx(1:n_cubes);

%% ======================== DISPLAY WORKSPACE ========================

fprintf('========================================================\n');
fprintf('  General Task Planner (Task 2a)\n');
fprintf('========================================================\n\n');

fprintf('--- Workspace ---\n');
fprintf('  Holders:  %d positions (occupied = cube starts here)\n', size(cfg.holders, 1));
for h = 1:size(cfg.holders, 1)
    if holder_occupied(h)
        fprintf('    H%d: (%.0f, %.0f) [occupied]\n', h, cfg.holders(h,:));
    else
        fprintf('    H%d: (%.0f, %.0f) [empty]\n', h, cfg.holders(h,:));
    end
end
fprintf('  Cubes: pick pos -> place at empty holder, offset [radial, tangent]\n');
for c = 1:n_cubes
    place_xy = cfg.holders(empty_holder_idx(c), :);
    off = cfg.cube_place_offsets(c, :);
    fprintf('    C%d: (%.0f, %.0f) -> (%.0f, %.0f) H%d  offset [%.1f, %.1f]\n', ...
        c, cfg.cubes(c,1), cfg.cubes(c,2), place_xy(1), place_xy(2), empty_holder_idx(c), off(1), off(2));
end
fprintf('  Place offset tuning: (%.1f, %.1f)\n', cfg.place_offset_tuning);
fprintf('\n');

%% ======================== PLAN SEQUENCE ========================

plan = {};
for ci = 1:n_cubes
    cube_xy = cfg.cubes(ci, 1:2);
    place_xy = cfg.holders(empty_holder_idx(ci), :);
    pick_angle = inferPickupAngle(cube_xy);
    release_z_adjust = 0;
    if isfield(cfg, 'cube_release_z_adjust') && ci <= numel(cfg.cube_release_z_adjust)
        release_z_adjust = cfg.cube_release_z_adjust(ci);
    end

    plan{end+1} = struct('action', 'pick', 'cube', ci, ...
        'cube_xy', cube_xy, 'for_rotation', false, ...
        'pickup_angle_deg', pick_angle, ...
        'desc', sprintf('C%d: Pick from (%.0f, %.0f) %.1f°', ci, cube_xy, pick_angle)); %#ok<AGROW>

    plan{end+1} = struct('action', 'place_target', 'cube', ci, ...
        'target_xy', place_xy, 'stack_level', 0, 'is_rotated', false, ...
        'pickup_angle_deg', pick_angle, 'release_z_adjust', release_z_adjust, ...
        'desc', sprintf('C%d: Place at (%.0f, %.0f) [empty holder H%d]', ci, place_xy, empty_holder_idx(ci))); %#ok<AGROW>
end

%% ======================== PRINT PLAN ========================

fprintf('--- Action Plan (%d steps) ---\n\n', length(plan));
for s = 1:length(plan)
    fprintf('  Step %2d: [%-14s] %s\n', s, plan{s}.action, plan{s}.desc);
end
fprintf('\n');

%% ======================== IK/FK VALIDATION ========================

fprintf('--- IK/FK Validation ---\n\n');

val_waypoints = collectWaypoints(plan, cfg);
max_fk_err = 0;
all_valid  = true;

fprintf('  %-4s  %-40s  %-24s  %-28s  %s\n', '#', 'Description', 'Target [x,y,z]', 'IK [q1..q4]', 'FK Err');
fprintf('  %s\n', repmat('-', 1, 120));

for i = 1:size(val_waypoints, 1)
    x = val_waypoints(i, 1);
    y = val_waypoints(i, 2);
    z = val_waypoints(i, 3);
    p = val_waypoints(i, 4);

    q = OpenManipulator.IK(x, y, z, p);
    [is_valid, violations] = OpenManipulator.JointLimits.Validate(q);
    [T_ee, ~] = OpenManipulator.FK(q);
    fk_pos = T_ee(1:3, 4)';
    fk_err = norm(fk_pos - [x, y, z]);
    max_fk_err = max(max_fk_err, fk_err);

    if ~is_valid || fk_err > 5.0
        status = 'X';
        all_valid = false;
    else
        status = ' ';
    end

    fprintf('  %s%-3d  %-40s  [%7.1f,%7.1f,%7.1f]  [%6.1f,%6.1f,%6.1f,%6.1f]  %.2f mm\n', ...
        status, i, '', x, y, z, q(1), q(2), q(3), q(4), fk_err);

    if ~is_valid
        for v = 1:length(violations)
            viol = violations{v};
            fprintf('    ! Joint %d (%s): %.1f deg exceeds [%.0f, %.0f]\n', ...
                viol.joint, viol.name, viol.angle, viol.min, viol.max);
        end
    end
end

fprintf('\n  Max FK Error: %.3f mm\n', max_fk_err);
if all_valid
    fprintf('  All waypoints valid.\n\n');
else
    fprintf('  WARNING: Some waypoints have issues — review above.\n\n');
end

if cfg.DRY_RUN
    fprintf('=== DRY_RUN mode — skipping hardware. ===\n');
    return;
end

%% ======================== CONFIRM ========================

resp = input('Proceed with hardware execution? (y/n) [y]: ', 's');
if ~isempty(strtrim(resp)) && ~strcmpi(strtrim(resp), 'y')
    fprintf('Aborted.\n');
    return;
end

%% ======================== HARDWARE EXECUTION ========================

fprintf('\n--- Connecting to Hardware ---\n\n');

hw = OpenManipulator.HardwareInterface(cfg.PORT, cfg.BAUD);
cleanup = onCleanup(@() safeShutdown(hw, cfg));

hw.configure(cfg.VELOCITY);
hw.enableTorque();
hw.openGripper();
pause(0.25);

fprintf('Moving to HOME...\n');
hw.moveToPose(cfg.HOME_POSE(1), cfg.HOME_POSE(2), cfg.HOME_POSE(3), cfg.HOME_POSE(4), ...
              cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

fprintf('\n========================================================\n');
fprintf('  Executing Plan (%d steps)\n', length(plan));
fprintf('========================================================\n\n');

for s = 1:length(plan)
    step = plan{s};
    fprintf('\n== Step %d/%d: %s ==\n', s, length(plan), step.desc);

    switch step.action
        case 'pick'
            pickCube(hw, step.cube_xy, step.for_rotation, step.pickup_angle_deg, cfg, step.cube);

        case 'place_target'
            fprintf('  [planner] place_target: cube_index=%d  target_xy=(%.1f, %.1f)\n', step.cube, step.target_xy(1), step.target_xy(2));
            placeCube(hw, step.target_xy, step.stack_level, step.is_rotated, step.pickup_angle_deg, cfg, step.release_z_adjust, step.cube);

        otherwise
            fprintf('  Unknown action: %s — skipping.\n', step.action);
    end

    if strcmp(step.action, 'place_target')
        fprintf('    Returning to HOME...\n');
        hw.moveToPose(cfg.HOME_POSE(1), cfg.HOME_POSE(2), cfg.HOME_POSE(3), cfg.HOME_POSE(4), ...
                      cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
        pause(0.25);
    end
end

%% ======================== SHUTDOWN ========================

fprintf('\n========================================================\n');
fprintf('  Task Complete — Returning Home\n');
fprintf('========================================================\n\n');

hw.moveToPose(cfg.HOME_POSE(1), cfg.HOME_POSE(2), cfg.HOME_POSE(3), cfg.HOME_POSE(4), ...
    cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);
hw.disconnect();
fprintf('=== Done ===\n');


%% ======================== LOCAL FUNCTIONS ========================

function safeShutdown(hw, cfg)
    fprintf('\n>>> Emergency cleanup triggered...\n');
    try hw.openGripper(); catch, end
    try
        hw.moveToPose(cfg.HOME_POSE(1), cfg.HOME_POSE(2), cfg.HOME_POSE(3), cfg.HOME_POSE(4), ...
                      cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
    catch
    end
    try hw.disconnect(); catch, end
    fprintf('>>> Safe shutdown complete.\n');
end

function angle_deg = inferPickupAngle(xy)
    raw = atan2d(abs(xy(2)), xy(1));
    buckets = [0, 22.5, 45];
    [~, idx] = min(abs(buckets - raw));
    angle_deg = buckets(idx);
end

function off = getPickOffsetStdForAngle(cfg, angle_deg)
    if abs(angle_deg - 0) < 1
        off = cfg.PICK_OFFSET_STD_0;
    elseif abs(angle_deg - 22.5) < 1
        off = cfg.PICK_OFFSET_STD_22_5;
    elseif abs(angle_deg - 45) < 1
        off = cfg.PICK_OFFSET_STD_45;
    else
        off = [0, 0];
    end
end

function waypoints = collectWaypoints(plan, cfg)
    waypoints = [];
    pick_z  = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2 + cfg.PICK_Z_OFFSET_MM;
    hover_z = cfg.HOVER_Z;
    place_approach_z = cfg.PLACE_APPROACH_Z;

    for s = 1:length(plan)
        step = plan{s};
        switch step.action
            case 'pick'
                px = step.cube_xy(1); py = step.cube_xy(2);
                if isfield(cfg, 'cube_pick_offsets') && step.cube >= 1 && step.cube <= size(cfg.cube_pick_offsets, 1)
                    off = cfg.cube_pick_offsets(step.cube, :);
                else
                    off = getPickOffsetStdForAngle(cfg, step.pickup_angle_deg);
                end
                brg = atan2(py, px);
                px = px + off(1)*cos(brg) - off(2)*sin(brg);
                py = py + off(1)*sin(brg) + off(2)*cos(brg);
                waypoints(end+1,:) = [px, py, hover_z, -90]; %#ok<AGROW>
                waypoints(end+1,:) = [px, py, pick_z, -90]; %#ok<AGROW>
                waypoints(end+1,:) = [px, py, pick_z + cfg.PICK_LIFT_MM, -90]; %#ok<AGROW>
                waypoints(end+1,:) = [px, py, place_approach_z, -90]; %#ok<AGROW>

            case 'place_target'
                tx = step.target_xy(1); ty = step.target_xy(2);
                sl = step.stack_level;
                pz = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE/2 + cfg.PICK_Z_OFFSET_MM + sl * cfg.CUBE_SIZE;
                % Per-cube offset [radial, tangential]
                po = cfg.cube_place_offsets(step.cube, :);
                brg_t = atan2(ty, tx);
                po_wx = po(1)*cos(brg_t) - po(2)*sin(brg_t);
                po_wy = po(1)*sin(brg_t) + po(2)*cos(brg_t);
                ox = po_wx + cfg.place_offset_tuning(1);
                oy = po_wy + cfg.place_offset_tuning(2);
                ax = tx + ox; ay = ty + oy;
                pp = -90;
                rza = step.release_z_adjust;
                waypoints(end+1,:) = [ax, ay, place_approach_z, pp]; %#ok<AGROW>
                waypoints(end+1,:) = [ax, ay, pz + cfg.PLACE_VERTICAL_OFFSET_MM, pp]; %#ok<AGROW>
                waypoints(end+1,:) = [ax, ay, pz + cfg.PLACE_DROP_MM + rza, pp]; %#ok<AGROW>
                waypoints(end+1,:) = [ax, ay, place_approach_z, pp]; %#ok<AGROW>
        end
    end
end
