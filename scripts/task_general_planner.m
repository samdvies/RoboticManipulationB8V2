%% task_general_planner.m
% General Task Planner — Pick, Rotate, Bridge-Pick, and Stack 3 cubes.
%
% Loads workspace from task_config.m, plans a sequence of primitive
% actions, validates via IK/FK, then executes on hardware.
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

%% ======================== REORDER CUBES: STANDARD FIRST ========================
% Process standard (no bridge, no rotation) cubes first so they get a clean
% pick from HOME. Rotation cubes next, bridge cubes last.
under_bridge = (cfg.cubes(:, 3) == 1);
needs_rot    = (cfg.cubes(:, 4) == 1);
% Priority: 0 = standard (first), 1 = rotation only (second), 2 = bridge (last)
priority = 2 * under_bridge + 1 * (needs_rot & ~under_bridge);
[~, order] = sort(priority);
cfg.cubes = cfg.cubes(order, :);

fprintf('========================================================\n');
fprintf('  General Task Planner\n');
fprintf('========================================================\n\n');

%% ======================== DISPLAY WORKSPACE ========================

fprintf('--- Workspace (cubes reordered: standard first) ---\n');
fprintf('  Bridge:   (%.0f, %.0f)  clearance=%.0f mm\n', cfg.BRIDGE_POS, cfg.BRIDGE_CLEARANCE_Z);
fprintf('  Target:   (%.0f, %.0f)\n', cfg.target);
fprintf('  Holders:  %d positions\n', size(cfg.holders, 1));
for h = 1:size(cfg.holders, 1)
    fprintf('    H%d: (%.0f, %.0f)\n', h, cfg.holders(h,:));
end
fprintf('  Cubes (stacking order bottom->top):\n');
for c = 1:size(cfg.cubes, 1)
    flags = '';
    if cfg.cubes(c, 3), flags = [flags ' BRIDGE']; end %#ok<AGROW>
    if cfg.cubes(c, 4), flags = [flags ' ROTATE']; end %#ok<AGROW>
    if isempty(flags), flags = ' (standard)'; end
    fprintf('    C%d: (%.0f, %.0f)%s\n', c, cfg.cubes(c,1), cfg.cubes(c,2), flags);
end
fprintf('  Place offset tuning: (%.1f, %.1f)\n', cfg.place_offset_tuning);
fprintf('\n');

%% ======================== PLAN SEQUENCE ========================

% Track holder occupancy: a holder is occupied if a cube starts there
% or if the target is on a holder
holder_occupied = false(size(cfg.holders, 1), 1);
for h = 1:size(cfg.holders, 1)
    for c = 1:size(cfg.cubes, 1)
        if norm(cfg.cubes(c, 1:2) - cfg.holders(h, :)) < 5
            holder_occupied(h) = true;
        end
    end
end

% Build action plan as a cell array of structs
plan = {};
stack_level = 0;

for ci = 1:size(cfg.cubes, 1)
    cube_xy       = cfg.cubes(ci, 1:2);
    under_bridge  = cfg.cubes(ci, 3) == 1;
    needs_rotation = cfg.cubes(ci, 4) == 1;

    if under_bridge && needs_rotation
        % CASE A: bridge + rotation — stage via empty holder
        staging_idx = findNearestEmptyHolder(cfg.holders, holder_occupied, cube_xy, cfg.target);
        if isempty(staging_idx)
            error('No empty holder available for staging cube %d.', ci);
        end
        staging_xy = cfg.holders(staging_idx, :);

        plan{end+1} = struct('action', 'bridge_pick', 'cube', ci, ...
            'cube_xy', cube_xy, 'desc', sprintf('C%d: Bridge pick from (%.0f,%.0f)', ci, cube_xy)); %#ok<AGROW>

        pick_angle_bridge = inferPickupAngle(cube_xy);
        plan{end+1} = struct('action', 'place_staging', 'cube', ci, ...
            'target_xy', staging_xy, 'stack_level', 0, 'is_rotated', false, ...
            'pickup_angle_deg', pick_angle_bridge, 'release_z_adjust', 0, ...
            'desc', sprintf('C%d: Place on staging holder H%d (%.0f,%.0f)', ci, staging_idx, staging_xy)); %#ok<AGROW>

        % Mark staging holder occupied, mark original cube holder free
        holder_occupied(staging_idx) = true;
        for h = 1:size(cfg.holders, 1)
            if norm(cube_xy - cfg.holders(h,:)) < 5
                holder_occupied(h) = false;
            end
        end

        pick_angle_staging = inferPickupAngle(staging_xy);
        plan{end+1} = struct('action', 'pick', 'cube', ci, ...
            'cube_xy', staging_xy, 'for_rotation', true, ...
            'pickup_angle_deg', pick_angle_staging, ...
            'desc', sprintf('C%d: Re-pick from staging (%.0f,%.0f)', ci, staging_xy)); %#ok<AGROW>

        plan{end+1} = struct('action', 'rotate', 'cube', ci, ...
            'current_xy', staging_xy, ...
            'desc', sprintf('C%d: Rotate pitch -90 -> 0', ci)); %#ok<AGROW>

        % Free the staging holder after re-pick
        holder_occupied(staging_idx) = false;

        plan{end+1} = struct('action', 'place_target', 'cube', ci, ...
            'target_xy', cfg.target, 'stack_level', stack_level, 'is_rotated', true, ...
            'pickup_angle_deg', pick_angle_staging, 'release_z_adjust', -5, ...
            'desc', sprintf('C%d: Place on target (%.0f,%.0f) stack=%d rotated', ci, cfg.target, stack_level)); %#ok<AGROW>

    elseif under_bridge && ~needs_rotation
        % CASE B: bridge only — held at pitch 0 (forwards), place at pitch 0
        plan{end+1} = struct('action', 'bridge_pick', 'cube', ci, ...
            'cube_xy', cube_xy, 'desc', sprintf('C%d: Bridge pick from (%.0f,%.0f)', ci, cube_xy)); %#ok<AGROW>

        % Free the cube's holder
        for h = 1:size(cfg.holders, 1)
            if norm(cube_xy - cfg.holders(h,:)) < 5
                holder_occupied(h) = false;
            end
        end

        pick_angle_b = inferPickupAngle(cube_xy);
        plan{end+1} = struct('action', 'place_target', 'cube', ci, ...
            'target_xy', cfg.target, 'stack_level', stack_level, 'is_rotated', true, ...
            'pickup_angle_deg', pick_angle_b, 'release_z_adjust', 0, ...
            'desc', sprintf('C%d: Place on target (%.0f,%.0f) stack=%d pitch=0', ci, cfg.target, stack_level)); %#ok<AGROW>

    elseif ~under_bridge && needs_rotation
        % CASE C: rotation only
        pick_angle = inferPickupAngle(cube_xy);
        plan{end+1} = struct('action', 'pick', 'cube', ci, ...
            'cube_xy', cube_xy, 'for_rotation', true, ...
            'pickup_angle_deg', pick_angle, ...
            'desc', sprintf('C%d: Pick from (%.0f,%.0f) %.1f°', ci, cube_xy, pick_angle)); %#ok<AGROW>

        % Free the cube's holder
        for h = 1:size(cfg.holders, 1)
            if norm(cube_xy - cfg.holders(h,:)) < 5
                holder_occupied(h) = false;
            end
        end

        plan{end+1} = struct('action', 'rotate', 'cube', ci, ...
            'current_xy', cube_xy, ...
            'desc', sprintf('C%d: Rotate pitch -90 -> 0', ci)); %#ok<AGROW>

        % Place offset links to pick angle (cube was picked at pick_angle)
        plan{end+1} = struct('action', 'place_target', 'cube', ci, ...
            'target_xy', cfg.target, 'stack_level', stack_level, 'is_rotated', true, ...
            'pickup_angle_deg', pick_angle, 'release_z_adjust', -5, ...
            'desc', sprintf('C%d: Place on target (%.0f,%.0f) stack=%d rotated', ci, cfg.target, stack_level)); %#ok<AGROW>

    else
        % CASE D: standard pick and place
        pick_angle = inferPickupAngle(cube_xy);
        plan{end+1} = struct('action', 'pick', 'cube', ci, ...
            'cube_xy', cube_xy, 'for_rotation', false, ...
            'pickup_angle_deg', pick_angle, ...
            'desc', sprintf('C%d: Pick from (%.0f,%.0f) %.1f°', ci, cube_xy, pick_angle)); %#ok<AGROW>

        % Free the cube's holder
        for h = 1:size(cfg.holders, 1)
            if norm(cube_xy - cfg.holders(h,:)) < 5
                holder_occupied(h) = false;
            end
        end

        plan{end+1} = struct('action', 'place_target', 'cube', ci, ...
            'target_xy', cfg.target, 'stack_level', stack_level, 'is_rotated', false, ...
            'pickup_angle_deg', pick_angle, 'release_z_adjust', 0, ...
            'desc', sprintf('C%d: Place on target (%.0f,%.0f) stack=%d', ci, cfg.target, stack_level)); %#ok<AGROW>
    end

    stack_level = stack_level + 1;
end

%% ======================== PRINT PLAN ========================

fprintf('--- Action Plan (%d steps) ---\n\n', length(plan));
for s = 1:length(plan)
    fprintf('  Step %2d: [%-14s] %s\n', s, plan{s}.action, plan{s}.desc);
end
fprintf('\n');

%% ======================== IK/FK VALIDATION ========================

fprintf('--- IK/FK Validation ---\n\n');

% Collect all waypoints that will be visited for validation
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
        case 'bridge_pick'
            bridgePickCube(hw, step.cube_xy, cfg);

        case 'pick'
            pickCube(hw, step.cube_xy, step.for_rotation, step.pickup_angle_deg, cfg);

        case 'rotate'
            rotateCubeInHand(hw, step.current_xy, cfg);

        case 'place_staging'
            placeCube(hw, step.target_xy, step.stack_level, step.is_rotated, step.pickup_angle_deg, cfg, step.release_z_adjust);

        case 'place_target'
            placeCube(hw, step.target_xy, step.stack_level, step.is_rotated, step.pickup_angle_deg, cfg, step.release_z_adjust);

        otherwise
            fprintf('  Unknown action: %s — skipping.\n', step.action);
    end

    % Go home between cube operations (after final place for each cube)
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

function idx = findNearestEmptyHolder(holders, occupied, ref_xy, target_xy)
    % Find the closest empty holder to ref_xy that is NOT the target position.
    idx = [];
    best_dist = Inf;
    for h = 1:size(holders, 1)
        if occupied(h), continue; end
        if norm(holders(h,:) - target_xy) < 5, continue; end
        d = norm(holders(h,:) - ref_xy);
        if d < best_dist
            best_dist = d;
            idx = h;
        end
    end
end

function angle_deg = inferPickupAngle(xy)
    % Infer pickup angle (0, 22.5, or 45) from pick coordinates.
    % Computes bearing from origin: atan2d(|y|, x), snapped to nearest bucket.
    raw = atan2d(abs(xy(2)), xy(1));
    buckets = [0, 22.5, 45];
    [~, idx] = min(abs(buckets - raw));
    angle_deg = buckets(idx);
end

function off = getPickOffsetForAngle(cfg, angle_deg)
    % Return [radial, tangential] mm for the given pickup angle (0, 22.5, or 45). Caller converts to world frame.
    if abs(angle_deg - 0) < 1
        off = cfg.PICK_OFFSET_0;
    elseif abs(angle_deg - 22.5) < 1
        off = cfg.PICK_OFFSET_22_5;
    elseif abs(angle_deg - 45) < 1
        off = cfg.PICK_OFFSET_45;
    else
        off = [0, 0];
    end
end

function off = getPlaceOffsetForAngle(cfg, angle_deg)
    % Return [radial, tangential] mm place offset for the given pick angle (0, 22.5, or 45). Caller converts to world frame.
    if abs(angle_deg - 0) < 1
        off = cfg.PLACE_OFFSET_0;
    elseif abs(angle_deg - 22.5) < 1
        off = cfg.PLACE_OFFSET_22_5;
    elseif abs(angle_deg - 45) < 1
        off = cfg.PLACE_OFFSET_45;
    else
        off = [0, 0];
    end
end

function off = getPickOffsetStdForAngle(cfg, angle_deg)
    % Return [radial, tangential] mm for standard (non-rotation) pick at the given angle (0, 22.5, or 45).
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

function off = getPlaceOffsetStdForAngle(cfg, angle_deg)
    % Return [radial, tangential] mm place offset for standard (non-rotated) place at the given angle (0, 22.5, or 45).
    if abs(angle_deg - 0) < 1
        off = cfg.PLACE_OFFSET_STD_0;
    elseif abs(angle_deg - 22.5) < 1
        off = cfg.PLACE_OFFSET_STD_22_5;
    elseif abs(angle_deg - 45) < 1
        off = cfg.PLACE_OFFSET_STD_45;
    else
        off = [0, 0];
    end
end

function waypoints = collectWaypoints(plan, cfg)
    % Build an Nx4 matrix of [x, y, z, pitch] for all waypoints in the plan
    % for IK/FK validation.
    waypoints = [];
    pick_z  = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2 + cfg.PICK_Z_OFFSET_MM;
    hover_z = cfg.HOVER_Z;
    place_approach_z = cfg.PLACE_APPROACH_Z;
    clear_z = cfg.BRIDGE_CLEARANCE_Z;
    bridge_x = cfg.BRIDGE_POS(1);
    approach_x = bridge_x - cfg.BRIDGE_APPROACH_OFFSET;

    for s = 1:length(plan)
        step = plan{s};
        switch step.action
            case 'bridge_pick'
                % Use same BridgeAvoidance planning as bridgePickCube for validation
                cx = step.cube_xy(1); cy = step.cube_xy(2);
                pitch_bridge = 0;
                BRIDGE_HALF_WIDTH_X = 5.0; BRIDGE_Z_MAX = 105.0; BRIDGE_GAP_Y = 50.0;
                BRIDGE_Y_MIN = -35.0; BRIDGE_Y_MAX = 35.0; BRIDGE_Z_MIN = 90.0;
                X_SCAN_MIN = 80.0; X_FRONT_OFFSET = 30.0; X_RETREAT_OFFSET = 40.0;
                bx_min = bridge_x - BRIDGE_HALF_WIDTH_X; bx_max = bridge_x + BRIDGE_HALF_WIDTH_X;
                bridge_zone = OpenManipulator.BridgeAvoidance.NewZone( ...
                    bx_min, bx_max, BRIDGE_Y_MIN, BRIDGE_Y_MAX, BRIDGE_Z_MIN, BRIDGE_Z_MAX);
                bridge_zones = OpenManipulator.BridgeAvoidance.BuildBridgeZones(bridge_zone, BRIDGE_GAP_Y, 10.0, 10.0);
                home_pose = [cfg.HOME_POSE(1), cfg.HOME_POSE(2), cfg.HOME_POSE(3), cfg.HOME_POSE(4)];
                pick_pose_b = [cx, cy, pick_z, pitch_bridge];
                lift_after_b = [cx, cy, pick_z + cfg.PICK_LIFT_MM, pitch_bridge];
                exit_target_b = [approach_x, cy, place_approach_z, pitch_bridge];
                planner_opts_b = struct('pitch_tolerance_deg', 5.0, 'vertical_clearance_mm', 30.0, 'samples', 20, ...
                    'x_scan_min', X_SCAN_MIN, 'x_front_offset', X_FRONT_OFFSET, 'x_retreat_offset', X_RETREAT_OFFSET);
                wp_entry_b = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints(home_pose, pick_pose_b, bridge_zone, bridge_zones, planner_opts_b);
                wp_exit_b = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints(lift_after_b, exit_target_b, bridge_zone, bridge_zones, planner_opts_b);
                for i = 1:size(wp_entry_b, 1), waypoints(end+1,:) = wp_entry_b(i,:); end %#ok<AGROW>
                waypoints(end+1,:) = pick_pose_b; %#ok<AGROW>
                waypoints(end+1,:) = lift_after_b; %#ok<AGROW>
                for i = 1:size(wp_exit_b, 1), waypoints(end+1,:) = wp_exit_b(i,:); end %#ok<AGROW>

            case 'pick'
                px = step.cube_xy(1); py = step.cube_xy(2);
                if step.for_rotation
                    off = getPickOffsetForAngle(cfg, step.pickup_angle_deg);
                else
                    off = getPickOffsetStdForAngle(cfg, step.pickup_angle_deg);
                end
                brg = atan2(py, px);
                px = px + off(1)*cos(brg) - off(2)*sin(brg);
                py = py + off(1)*sin(brg) + off(2)*cos(brg);
                waypoints(end+1,:) = [px, py, hover_z, -90]; %#ok<AGROW>
                waypoints(end+1,:) = [px, py, pick_z, -90]; %#ok<AGROW>
                waypoints(end+1,:) = [px, py, pick_z + cfg.PICK_LIFT_MM, -90]; %#ok<AGROW>
                waypoints(end+1,:) = [px, py, place_approach_z, -90]; %#ok<AGROW>  % fly height (bridge clearance)

            case 'rotate'
                cx = step.current_xy(1); cy = step.current_xy(2);
                r = sqrt(cx^2 + cy^2);
                if r < cfg.MIN_ROTATE_RADIUS && r > 0.001
                    sc = cfg.MIN_ROTATE_RADIUS / r;
                    waypoints(end+1,:) = [cx*sc, cy*sc, hover_z, -90]; %#ok<AGROW>
                    waypoints(end+1,:) = [cx*sc, cy*sc, hover_z, 0]; %#ok<AGROW>
                    waypoints(end+1,:) = [cx, cy, hover_z, 0]; %#ok<AGROW>
                else
                    waypoints(end+1,:) = [cx, cy, hover_z, 0]; %#ok<AGROW>
                end
                waypoints(end+1,:) = [cx, cy, place_approach_z, 0]; %#ok<AGROW>  % travel height to clear bridge

            case {'place_staging', 'place_target'}
                tx = step.target_xy(1); ty = step.target_xy(2);
                sl = step.stack_level;
                pz = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE/2 + cfg.PICK_Z_OFFSET_MM + sl * cfg.CUBE_SIZE;
                if step.is_rotated
                    r_p = sqrt(tx^2 + ty^2);
                    if r_p > 1e-6
                        ox = -cfg.PLACE_OFFSET_MAG * tx / r_p;
                        oy = -cfg.PLACE_OFFSET_MAG * ty / r_p;
                    else
                        ox = 0;
                        oy = 0;
                    end
                    po = getPlaceOffsetForAngle(cfg, step.pickup_angle_deg);
                else
                    ox = 0;
                    oy = 0;
                    po = getPlaceOffsetStdForAngle(cfg, step.pickup_angle_deg);
                end
                brg_t = atan2(ty, tx);
                po_wx = po(1)*cos(brg_t) - po(2)*sin(brg_t);
                po_wy = po(1)*sin(brg_t) + po(2)*cos(brg_t);
                ox = ox + po_wx;
                oy = oy + po_wy;
                ox = ox + cfg.place_offset_tuning(1);
                oy = oy + cfg.place_offset_tuning(2);
                ax = tx + ox; ay = ty + oy;
                if step.is_rotated, pp = 0; else, pp = -90; end
                place_z_approach = place_approach_z;
                if ~step.is_rotated && isfield(cfg, 'PLACE_APPROACH_Z_STANDARD')
                    place_z_approach = cfg.PLACE_APPROACH_Z_STANDARD;
                end
                waypoints(end+1,:) = [ax, ay, place_z_approach, pp]; %#ok<AGROW>
                rza = step.release_z_adjust;
                waypoints(end+1,:) = [ax, ay, pz + cfg.PLACE_VERTICAL_OFFSET_MM, pp]; %#ok<AGROW>  % directly above (straight-down approach)
                waypoints(end+1,:) = [ax, ay, pz + cfg.PLACE_DROP_MM + rza, pp]; %#ok<AGROW>  % release just above stack (drop, adjusted)
                waypoints(end+1,:) = [ax, ay, place_z_approach, pp]; %#ok<AGROW>
        end
    end
end
