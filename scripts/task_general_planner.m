%% task_general_planner.m
% General Task Planner - Task 2a: three cubes to three empty holders.
%
% Standard cubes use the existing pitch -90 pick/place flow.
% "Hard" cubes use a dedicated pitch 0 pick/place flow.
%
% Planning rules:
% - Hard cubes are prioritized.
% - A hard cube is placed into the furthest currently empty holder.
% - If a hard cube has a closer cube on the same radial line, that blocker
%   is moved first. Soft blockers go to the nearest empty holder.
% - Non-blocking soft cubes keep the original "first empty holder" policy.
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
cube_positions = cfg.cubes(:, 1:2);
cube_is_hard = resolveCubeHardFlags(cfg);
n_cubes = size(cube_positions, 1);

%% ======================== HOLDER OCCUPANCY ========================

holder_occupied = false(size(cfg.holders, 1), 1);
for h = 1:size(cfg.holders, 1)
    for c = 1:n_cubes
        if norm(cube_positions(c, :) - cfg.holders(h, :)) < 5
            holder_occupied(h) = true;
            break;
        end
    end
end

empty_holder_idx = find(~holder_occupied);
if numel(empty_holder_idx) < n_cubes
    error(['Not enough empty holders: %d cubes but only %d empty holders. ' ...
        'Ensure exactly three of the six holders are empty at start.'], ...
        n_cubes, numel(empty_holder_idx));
end

%% ======================== PLAN SEQUENCE ========================

plan = {};
holder_occupied_plan = holder_occupied;
assigned_holder_idx = zeros(n_cubes, 1);

use_forced = isfield(cfg, 'cube_forced_order') && isfield(cfg, 'cube_forced_targets');

if use_forced
    %% --- Forced order / explicit targets ---
    forced_order = cfg.cube_forced_order;
    forced_targets = cfg.cube_forced_targets;

    for fi = 1:numel(forced_order)
        cube_idx = forced_order(fi);
        cube_xy = cube_positions(cube_idx, :);
        pick_angle = inferPickupAngle(cube_xy);
        release_z_adjust = resolveReleaseZAdjust(cfg, cube_idx);
        is_hard = cube_is_hard(cube_idx);

        place_xy = forced_targets(cube_idx, :);

        % Find the holder index that matches the forced target coordinate
        place_idx = findSourceHolder(cfg.holders, place_xy);
        if isempty(place_idx)
            error('Forced target (%.1f, %.1f) for cube %d is not a known holder.', ...
                place_xy(1), place_xy(2), cube_idx);
        end
        assigned_holder_idx(cube_idx) = place_idx;

        if is_hard
            pick_desc = sprintf('C%d: Hard pick from (%.0f, %.0f) pitch=0', ...
                cube_idx, cube_xy(1), cube_xy(2));
        else
            pick_desc = sprintf('C%d: Pick from (%.0f, %.0f) %.1f deg', ...
                cube_idx, cube_xy(1), cube_xy(2), pick_angle);
        end

        place_desc = sprintf('C%d: Place at (%.0f, %.0f) [holder H%d]', ...
            cube_idx, place_xy(1), place_xy(2), place_idx);

        move_reason = 'forced';

        plan{end+1} = struct( ... %#ok<AGROW>
            'action', 'pick', ...
            'cube', cube_idx, ...
            'cube_xy', cube_xy, ...
            'pickup_angle_deg', pick_angle, ...
            'is_hard', is_hard, ...
            'move_reason', move_reason, ...
            'desc', pick_desc);

        plan{end+1} = struct( ... %#ok<AGROW>
            'action', 'place_target', ...
            'cube', cube_idx, ...
            'target_xy', place_xy, ...
            'stack_level', 0, ...
            'is_rotated', is_hard, ...
            'is_hard', is_hard, ...
            'pickup_angle_deg', pick_angle, ...
            'release_z_adjust', release_z_adjust, ...
            'holder_idx', place_idx, ...
            'desc', place_desc);

        source_holder_idx = findSourceHolder(cfg.holders, cube_xy);
        if isempty(source_holder_idx)
            error('Cube %d at (%.1f, %.1f) is not on a known holder.', ...
                cube_idx, cube_xy(1), cube_xy(2));
        end
        holder_occupied_plan(source_holder_idx) = false;
        holder_occupied_plan(place_idx) = true;
    end
else
    %% --- Autonomous planning (original logic) ---
    pending = 1:n_cubes;

    while ~isempty(pending)
        [cube_idx, move_reason] = chooseNextCube(pending, cube_positions, cube_is_hard);
        cube_xy = cube_positions(cube_idx, :);
        pick_angle = inferPickupAngle(cube_xy);
        release_z_adjust = resolveReleaseZAdjust(cfg, cube_idx);
        is_hard = cube_is_hard(cube_idx);
        remaining_pending = pending(pending ~= cube_idx);
        remaining_hard_targets = cube_positions(remaining_pending(cube_is_hard(remaining_pending)), :);

        if is_hard
            place_idx = findFurthestEligibleEmptyHolder( ...
                cfg.holders, holder_occupied_plan, remaining_hard_targets);
        elseif strcmp(move_reason, 'blocking_soft')
            place_idx = findNearestEligibleEmptyHolder( ...
                cfg.holders, holder_occupied_plan, cube_xy, remaining_hard_targets);
        else
            place_idx = findFirstEmptyHolder(holder_occupied_plan);
        end

        if isempty(place_idx)
            error('No empty holder available while planning cube %d.', cube_idx);
        end

        place_xy = cfg.holders(place_idx, :);
        assigned_holder_idx(cube_idx) = place_idx;

        if is_hard
            pick_desc = sprintf('C%d: Hard pick from (%.0f, %.0f) pitch=0', ...
                cube_idx, cube_xy(1), cube_xy(2));
        elseif strcmp(move_reason, 'blocking_soft')
            pick_desc = sprintf('C%d: Move blocking cube from (%.0f, %.0f)', ...
                cube_idx, cube_xy(1), cube_xy(2));
        else
            pick_desc = sprintf('C%d: Pick from (%.0f, %.0f) %.1f deg', ...
                cube_idx, cube_xy(1), cube_xy(2), pick_angle);
        end

        place_desc = sprintf('C%d: Place at (%.0f, %.0f) [holder H%d]', ...
            cube_idx, place_xy(1), place_xy(2), place_idx);

        plan{end+1} = struct( ... %#ok<AGROW>
            'action', 'pick', ...
            'cube', cube_idx, ...
            'cube_xy', cube_xy, ...
            'pickup_angle_deg', pick_angle, ...
            'is_hard', is_hard, ...
            'move_reason', move_reason, ...
            'desc', pick_desc);

        plan{end+1} = struct( ... %#ok<AGROW>
            'action', 'place_target', ...
            'cube', cube_idx, ...
            'target_xy', place_xy, ...
            'stack_level', 0, ...
            'is_rotated', is_hard, ...
            'is_hard', is_hard, ...
            'pickup_angle_deg', pick_angle, ...
            'release_z_adjust', release_z_adjust, ...
            'holder_idx', place_idx, ...
            'desc', place_desc);

        source_holder_idx = findSourceHolder(cfg.holders, cube_xy);
        if isempty(source_holder_idx)
            error('Cube %d at (%.1f, %.1f) is not on a known holder.', ...
                cube_idx, cube_xy(1), cube_xy(2));
        end
        holder_occupied_plan(source_holder_idx) = false;
        holder_occupied_plan(place_idx) = true;

        pending(pending == cube_idx) = [];
    end
end

%% ======================== DISPLAY WORKSPACE ========================

fprintf('========================================================\n');
fprintf('  General Task Planner (Task 2a)\n');
fprintf('========================================================\n\n');

fprintf('--- Workspace ---\n');
fprintf('  Holders: %d positions (occupied = cube starts here)\n', size(cfg.holders, 1));
for h = 1:size(cfg.holders, 1)
    if holder_occupied(h)
        fprintf('    H%d: (%.0f, %.0f) [occupied]\n', h, cfg.holders(h, :));
    else
        fprintf('    H%d: (%.0f, %.0f) [empty]\n', h, cfg.holders(h, :));
    end
end

fprintf('  Cubes:\n');
for c = 1:n_cubes
    hard_label = '';
    if cube_is_hard(c)
        hard_label = ' HARD';
    end
    place_xy = cfg.holders(assigned_holder_idx(c), :);
    off = cfg.cube_place_offsets(c, :);
    fprintf(['    C%d: (%.0f, %.0f)%s -> (%.0f, %.0f) H%d  ' ...
        'offset [%.1f, %.1f]\n'], ...
        c, cube_positions(c, 1), cube_positions(c, 2), hard_label, ...
        place_xy(1), place_xy(2), assigned_holder_idx(c), off(1), off(2));
end
fprintf('  Place offset tuning: (%.1f, %.1f)\n', cfg.place_offset_tuning);
fprintf('\n');

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
all_valid = true;

fprintf('  %-4s  %-40s  %-24s  %-28s  %s\n', ...
    '#', 'Description', 'Target [x,y,z]', 'IK [q1..q4]', 'FK Err');
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
    fprintf('  WARNING: Some waypoints have issues - review above.\n\n');
end

if cfg.DRY_RUN
    fprintf('=== DRY_RUN mode - skipping hardware. ===\n');
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
cleanup = onCleanup(@() safeShutdown(hw, cfg)); %#ok<NASGU>

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
            if step.is_hard
                pickCubeHard(hw, step.cube_xy, step.pickup_angle_deg, cfg, step.cube);
            else
                pickCube(hw, step.cube_xy, false, step.pickup_angle_deg, cfg, step.cube);
            end

        case 'place_target'
            fprintf('  [planner] place_target: cube_index=%d  target_xy=(%.1f, %.1f)\n', ...
                step.cube, step.target_xy(1), step.target_xy(2));
            placeCube(hw, step.target_xy, step.stack_level, step.is_rotated, ...
                step.pickup_angle_deg, cfg, step.release_z_adjust, step.cube);

        otherwise
            fprintf('  Unknown action: %s - skipping.\n', step.action);
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
fprintf('  Task Complete - Returning Home\n');
fprintf('========================================================\n\n');

hw.moveToPose(cfg.HOME_POSE(1), cfg.HOME_POSE(2), cfg.HOME_POSE(3), cfg.HOME_POSE(4), ...
    cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);
hw.disconnect();
fprintf('=== Done ===\n');

%% ======================== LOCAL FUNCTIONS ========================

function safeShutdown(hw, cfg)
    fprintf('\n>>> Emergency cleanup triggered...\n');
    try
        hw.openGripper();
    catch
    end
    try
        hw.moveToPose(cfg.HOME_POSE(1), cfg.HOME_POSE(2), cfg.HOME_POSE(3), cfg.HOME_POSE(4), ...
            cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
    catch
    end
    try
        hw.disconnect();
    catch
    end
    fprintf('>>> Safe shutdown complete.\n');
end

function hard_flags = resolveCubeHardFlags(cfg)
    num_cubes = size(cfg.cubes, 1);
    hard_flags = false(num_cubes, 1);
    if size(cfg.cubes, 2) >= 3
        hard_flags = cfg.cubes(:, 3) ~= 0;
    end
end

function [cube_idx, move_reason] = chooseNextCube(pending, cube_positions, cube_is_hard)
    hard_pending = pending(cube_is_hard(pending));
    cube_idx = [];
    move_reason = '';

    if ~isempty(hard_pending)
        hard_radii = sqrt(sum(cube_positions(hard_pending, :) .^ 2, 2));
        [~, order] = sort(hard_radii, 'descend');
        hard_pending = hard_pending(order);

        fallback_cube = [];
        fallback_reason = '';

        for i = 1:numel(hard_pending)
            hard_cube = hard_pending(i);
            blocker = findBlockingCube(hard_cube, pending, cube_positions);
            if isempty(blocker)
                cube_idx = hard_cube;
                move_reason = 'hard';
                return;
            end

            if isempty(fallback_cube)
                fallback_cube = blocker;
                if cube_is_hard(blocker)
                    fallback_reason = 'blocking_hard';
                else
                    fallback_reason = 'blocking_soft';
                end
            end
        end

        if ~isempty(fallback_cube)
            cube_idx = fallback_cube;
            move_reason = fallback_reason;
            return;
        end
    end

    cube_idx = pending(1);
    move_reason = 'soft';
end

function blocker = findBlockingCube(target_cube, pending, cube_positions)
    target_xy = cube_positions(target_cube, :);
    target_radius = norm(target_xy);
    blocker = [];
    blocker_radius = -Inf;

    for idx = pending
        if idx == target_cube
            continue;
        end

        candidate_xy = cube_positions(idx, :);
        candidate_radius = norm(candidate_xy);
        if candidate_radius >= target_radius
            continue;
        end

        if ~isSameRadialLine(candidate_xy, target_xy)
            continue;
        end

        if candidate_radius > blocker_radius
            blocker = idx;
            blocker_radius = candidate_radius;
        end
    end
end

function tf = isSameRadialLine(a_xy, b_xy)
    cross_val = a_xy(1) * b_xy(2) - a_xy(2) * b_xy(1);
    dot_val = a_xy(1) * b_xy(1) + a_xy(2) * b_xy(2);
    tf = abs(cross_val) < 1e-9 && dot_val > 0;
end

function holder_idx = findSourceHolder(holders, cube_xy)
    holder_idx = [];
    for h = 1:size(holders, 1)
        if norm(holders(h, :) - cube_xy) < 5
            holder_idx = h;
            return;
        end
    end
end

function holder_idx = findFirstEmptyHolder(holder_occupied)
    holder_idx = find(~holder_occupied, 1, 'first');
end

function holder_idx = findNearestEmptyHolder(holders, holder_occupied, ref_xy)
    holder_idx = [];
    best_dist = Inf;
    for h = 1:size(holders, 1)
        if holder_occupied(h)
            continue;
        end

        dist = norm(holders(h, :) - ref_xy);
        if dist < best_dist
            best_dist = dist;
            holder_idx = h;
        end
    end
end

function holder_idx = findFurthestEmptyHolder(holders, holder_occupied)
    holder_idx = [];
    best_radius = -Inf;
    for h = 1:size(holders, 1)
        if holder_occupied(h)
            continue;
        end

        radius = norm(holders(h, :));
        if radius > best_radius
            best_radius = radius;
            holder_idx = h;
        end
    end
end

function holder_idx = findNearestEligibleEmptyHolder(holders, holder_occupied, ref_xy, hard_targets_xy)
    holder_idx = [];
    best_dist = Inf;
    for h = 1:size(holders, 1)
        if holder_occupied(h)
            continue;
        end
        if wouldBlockAnyHardPick(holders(h, :), hard_targets_xy)
            continue;
        end

        dist = norm(holders(h, :) - ref_xy);
        if dist < best_dist
            best_dist = dist;
            holder_idx = h;
        end
    end
end

function holder_idx = findFurthestEligibleEmptyHolder(holders, holder_occupied, hard_targets_xy)
    holder_idx = [];
    best_radius = -Inf;
    for h = 1:size(holders, 1)
        if holder_occupied(h)
            continue;
        end
        if wouldBlockAnyHardPick(holders(h, :), hard_targets_xy)
            continue;
        end

        radius = norm(holders(h, :));
        if radius > best_radius
            best_radius = radius;
            holder_idx = h;
        end
    end
end

function tf = wouldBlockAnyHardPick(candidate_xy, hard_targets_xy)
    tf = false;
    if isempty(hard_targets_xy)
        return;
    end

    for i = 1:size(hard_targets_xy, 1)
        hard_target_xy = hard_targets_xy(i, :);
        if isSameRadialLine(candidate_xy, hard_target_xy) && ...
                norm(candidate_xy) < norm(hard_target_xy)
            tf = true;
            return;
        end
    end
end

function value = resolveReleaseZAdjust(cfg, cube_idx)
    value = 0;
    if isfield(cfg, 'cube_release_z_adjust') && cube_idx <= numel(cfg.cube_release_z_adjust)
        value = cfg.cube_release_z_adjust(cube_idx);
    end
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
    pick_z = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2 + cfg.PICK_Z_OFFSET_MM;
    hover_z = cfg.HOVER_Z;
    place_approach_z = cfg.PLACE_APPROACH_Z;
    hard_approach_z = cfg.HARD_APPROACH_Z;
    hard_transit_pitch = cfg.HARD_TRANSIT_PITCH;

    for s = 1:length(plan)
        step = plan{s};
        switch step.action
            case 'pick'
                px = step.cube_xy(1);
                py = step.cube_xy(2);
                if isfield(cfg, 'cube_pick_offsets') && ...
                        step.cube >= 1 && step.cube <= size(cfg.cube_pick_offsets, 1)
                    off = cfg.cube_pick_offsets(step.cube, :);
                else
                    off = getPickOffsetStdForAngle(cfg, step.pickup_angle_deg);
                end

                bearing = atan2(py, px);
                px = px + off(1) * cos(bearing) - off(2) * sin(bearing);
                py = py + off(1) * sin(bearing) + off(2) * cos(bearing);

                if step.is_hard
                    waypoints(end+1, :) = [px, py, hard_approach_z, hard_transit_pitch]; %#ok<AGROW>
                    waypoints(end+1, :) = [px, py, pick_z, 0]; %#ok<AGROW>
                    waypoints(end+1, :) = [px, py, pick_z + cfg.PICK_LIFT_MM, 0]; %#ok<AGROW>
                    waypoints(end+1, :) = [px, py, hard_approach_z, hard_transit_pitch]; %#ok<AGROW>
                else
                    waypoints(end+1, :) = [px, py, hover_z, cfg.TRANSIT_PITCH]; %#ok<AGROW>
                    waypoints(end+1, :) = [px, py, pick_z, -90]; %#ok<AGROW>
                    waypoints(end+1, :) = [px, py, pick_z + cfg.PICK_LIFT_MM, -90]; %#ok<AGROW>
                    waypoints(end+1, :) = [px, py, place_approach_z, cfg.TRANSIT_PITCH]; %#ok<AGROW>
                end

            case 'place_target'
                tx = step.target_xy(1);
                ty = step.target_xy(2);
                sl = step.stack_level;
                pz = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2 + cfg.PICK_Z_OFFSET_MM + ...
                    sl * cfg.CUBE_SIZE;
                po = cfg.cube_place_offsets(step.cube, :);
                bearing_place = atan2(ty, tx);
                po_wx = po(1) * cos(bearing_place) - po(2) * sin(bearing_place);
                po_wy = po(1) * sin(bearing_place) + po(2) * cos(bearing_place);
                ox = po_wx + cfg.place_offset_tuning(1);
                oy = po_wy + cfg.place_offset_tuning(2);
                ax = tx + ox;
                ay = ty + oy;
                if step.is_hard
                    work_pitch = 0;
                    transit_pitch = 0;
                else
                    work_pitch = -90;
                    transit_pitch = cfg.TRANSIT_PITCH;
                end
                rza = step.release_z_adjust;
                waypoints(end+1, :) = [ax, ay, place_approach_z, transit_pitch]; %#ok<AGROW>
                waypoints(end+1, :) = [ax, ay, pz + cfg.PLACE_VERTICAL_OFFSET_MM, work_pitch]; %#ok<AGROW>
                waypoints(end+1, :) = [ax, ay, pz + cfg.PLACE_DROP_MM + rza, work_pitch]; %#ok<AGROW>
                waypoints(end+1, :) = [ax, ay, place_approach_z, transit_pitch]; %#ok<AGROW>
        end
    end
end
