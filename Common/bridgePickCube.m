function bridgePickCube(hw, cube_xy, cfg)
% BRIDGEPICKCUBE  Pick a cube from under the bridge (ported from master scripts/bridge_pick.m).
%
% Uses OpenManipulator.BridgeAvoidance to plan bridge-safe entry/exit waypoints,
% then executes them with preplanned_route context. Pitch 0 (forwards) for clearance.
%
% Inputs:
%   hw      - OpenManipulator.HardwareInterface instance
%   cube_xy - [x, y] position of cube under the bridge (mm)
%   cfg     - config struct from task_config()

PITCH_BRIDGE = 0;

% Bridge geometry (match master bridge_pick / BridgeAvoidance)
BRIDGE_HALF_WIDTH_X = 5.0;
BRIDGE_Z_MAX = 105.0;
BRIDGE_GAP_Y = 50.0;
BRIDGE_Y_MIN = -35.0;
BRIDGE_Y_MAX = 35.0;
BRIDGE_Z_MIN = 90.0;
X_SCAN_MIN = 80.0;
X_FRONT_OFFSET = 30.0;
X_RETREAT_OFFSET = 40.0;

bridge_center_x = cfg.BRIDGE_POS(1);
BRIDGE_X_MIN = bridge_center_x - BRIDGE_HALF_WIDTH_X;
BRIDGE_X_MAX = bridge_center_x + BRIDGE_HALF_WIDTH_X;

bridge_zone = OpenManipulator.BridgeAvoidance.NewZone( ...
    BRIDGE_X_MIN, BRIDGE_X_MAX, BRIDGE_Y_MIN, BRIDGE_Y_MAX, BRIDGE_Z_MIN, BRIDGE_Z_MAX);
bridge_zones = OpenManipulator.BridgeAvoidance.BuildBridgeZones(bridge_zone, BRIDGE_GAP_Y, 10.0, 10.0);

% Start from current pose (planner has us at HOME before bridge_pick)
q_current = hw.readAngles();
[T_current, ~] = OpenManipulator.FK(q_current);
start_pos = T_current(1:3, 4)';
start_pitch = -(q_current(2) + q_current(3) + q_current(4));
home_pose = [start_pos(1), start_pos(2), start_pos(3), start_pitch];

cube_x = cube_xy(1);
cube_y = cube_xy(2);
pick_z = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2;
pick_pose = [cube_x, cube_y, pick_z, PITCH_BRIDGE];
lift_after_pick_pose = [cube_x, cube_y, pick_z + cfg.PICK_LIFT_MM, PITCH_BRIDGE];

% Exit target: approach position at travel height (clear bridge, ready for place)
approach_x = bridge_center_x - cfg.BRIDGE_APPROACH_OFFSET;
exit_target_pose = [approach_x, cube_y, cfg.PLACE_APPROACH_Z, PITCH_BRIDGE];

planner_opts = struct('pitch_tolerance_deg', 5.0, ...
    'vertical_clearance_mm', 30.0, ...
    'samples', 20, ...
    'x_scan_min', X_SCAN_MIN, ...
    'x_front_offset', X_FRONT_OFFSET, ...
    'x_retreat_offset', X_RETREAT_OFFSET);

wp_entry = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
    home_pose, pick_pose, bridge_zone, bridge_zones, planner_opts);
wp_exit = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
    lift_after_pick_pose, exit_target_pose, bridge_zone, bridge_zones, planner_opts);

USE_DYNAMIC_PITCH = false;
MOVE_TIME = cfg.MOVE_TIME;
Z_FLOOR = cfg.Z_FLOOR;
MODE_EXEC = cfg.MOTION_MODE;

fprintf('    [bridge-pick] Entry waypoints: %d\n', size(wp_entry, 1));
fprintf('    [bridge-pick] Exit waypoints:  %d\n', size(wp_exit, 1));

% --- Phase 1: Entry route to pick ---
entry_ctx = struct('zones', bridge_zones, ...
    'final_target_pose', wp_entry(end, :), ...
    'preplanned_route', true, ...
    'dynamic_pitch', USE_DYNAMIC_PITCH);
for i = 1:size(wp_entry, 1)
    wp = wp_entry(i, :);
    fprintf('    [bridge-pick] Entry %d/%d -> [%.1f, %.1f, %.1f, %.1f]\n', ...
        i, size(wp_entry, 1), wp(1), wp(2), wp(3), wp(4));
    exec_mode_wp = MODE_EXEC;
    if i == size(wp_entry, 1)
        exec_mode_wp = 2;  % Task-space linear for final approach
    end
    hw.moveToPose(wp(1), wp(2), wp(3), wp(4), MOVE_TIME, exec_mode_wp, Z_FLOOR, entry_ctx);
    pause(0.3);
end
% Final lock at exact pick pose
hw.moveToPose(pick_pose(1), pick_pose(2), pick_pose(3), pick_pose(4), ...
    max(0.8, 0.5 * MOVE_TIME), 1, Z_FLOOR, entry_ctx);
pause(0.2);

fprintf('    [bridge-pick] Close gripper...\n');
hw.closeGripper();
pause(cfg.PICK_HOLD_TIME);

% --- Phase 2: Lift ---
lift_ctx = struct('zones', bridge_zones, ...
    'final_target_pose', lift_after_pick_pose, ...
    'preplanned_route', true, ...
    'dynamic_pitch', USE_DYNAMIC_PITCH);
hw.moveToPose(lift_after_pick_pose(1), lift_after_pick_pose(2), lift_after_pick_pose(3), lift_after_pick_pose(4), ...
    MOVE_TIME, MODE_EXEC, Z_FLOOR, lift_ctx);
pause(0.3);

% --- Phase 3: Exit route to travel height ---
exit_ctx = struct('zones', bridge_zones, ...
    'final_target_pose', wp_exit(end, :), ...
    'preplanned_route', true, ...
    'dynamic_pitch', USE_DYNAMIC_PITCH);
for i = 1:size(wp_exit, 1)
    wp = wp_exit(i, :);
    fprintf('    [bridge-pick] Exit %d/%d -> [%.1f, %.1f, %.1f, %.1f]\n', ...
        i, size(wp_exit, 1), wp(1), wp(2), wp(3), wp(4));
    hw.moveToPose(wp(1), wp(2), wp(3), wp(4), MOVE_TIME, MODE_EXEC, Z_FLOOR, exit_ctx);
    pause(0.3);
end

end
