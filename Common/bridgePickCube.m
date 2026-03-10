function bridgePickCube(hw, cube_xy, cfg)
% BRIDGEPICKCUBE  Pick a cube from under the bridge (ported from master scripts/bridge_pick.m).
%
% Uses OpenManipulator.BridgeAvoidance to plan bridge-safe entry/exit waypoints,
% then executes them with preplanned_route context. Pitch 0 (forwards) for clearance.
%
% No pick offsets (PICK_OFFSET_* / PICK_OFFSET_STD_*) are applied; nominal cube_xy is used.
%
% Inputs:
%   hw      - OpenManipulator.HardwareInterface instance
%   cube_xy - [x, y] position of cube under the bridge (mm)
%   cfg     - config struct from task_config()

PITCH_BRIDGE = 0;
BRIDGE_MOVE_TIME = 1.25;   % 2x speed (was 2.5)
Z_FLOOR = 10;
MODE_EXEC = 1;

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

% Fixed bridge-safe start pose at pitch 0 (matches working bridge_pick.m)
home_pose = [200, 0, 180, 0];

% Move to bridge start pose before planning entry
fprintf('    [bridge-pick] Moving to bridge start pose [200, 0, 180, 0]...\n');
hw.moveToPose(home_pose(1), home_pose(2), home_pose(3), home_pose(4), ...
    BRIDGE_MOVE_TIME, MODE_EXEC, Z_FLOOR);
pause(0.25);

% Set jaw to 50% for bridge clearance (fully open may collide)
fprintf('    [bridge-pick] Setting jaw to 50%% for bridge clearance...\n');
hw.setGripperPosition(50);
pause(0.4);

cube_x = cube_xy(1);
cube_y = cube_xy(2);
pick_z = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2 + cfg.PICK_Z_OFFSET_MM;
pick_pose = [cube_x, cube_y, pick_z, PITCH_BRIDGE];
lift_after_pick_pose = [cube_x, cube_y, pick_z + cfg.PICK_LIFT_MM, PITCH_BRIDGE];

planner_opts = struct('pitch_tolerance_deg', 5.0, ...
    'vertical_clearance_mm', 30.0, ...
    'samples', 20, ...
    'x_scan_min', X_SCAN_MIN, ...
    'x_front_offset', X_FRONT_OFFSET, ...
    'x_retreat_offset', X_RETREAT_OFFSET);

wp_entry = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
    home_pose, pick_pose, bridge_zone, bridge_zones, planner_opts);
wp_exit = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints( ...
    lift_after_pick_pose, home_pose, bridge_zone, bridge_zones, planner_opts);

USE_DYNAMIC_PITCH = false;

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
        exec_mode_wp = 2;
    end
    hw.moveToPose(wp(1), wp(2), wp(3), wp(4), BRIDGE_MOVE_TIME, exec_mode_wp, Z_FLOOR, entry_ctx);
    pause(0.15);
end
% Final lock at exact pick pose
hw.moveToPose(pick_pose(1), pick_pose(2), pick_pose(3), pick_pose(4), ...
    max(0.4, 0.5 * BRIDGE_MOVE_TIME), 1, Z_FLOOR, entry_ctx);
pause(0.1);

fprintf('    [bridge-pick] Close gripper...\n');
hw.closeGripper();
pause(cfg.PICK_HOLD_TIME);

% --- Phase 2: Lift ---
lift_ctx = struct('zones', bridge_zones, ...
    'final_target_pose', lift_after_pick_pose, ...
    'preplanned_route', true, ...
    'dynamic_pitch', USE_DYNAMIC_PITCH);
hw.moveToPose(lift_after_pick_pose(1), lift_after_pick_pose(2), lift_after_pick_pose(3), lift_after_pick_pose(4), ...
    BRIDGE_MOVE_TIME, MODE_EXEC, Z_FLOOR, lift_ctx);
pause(0.15);

% --- Phase 3: Exit route back to bridge start pose ---
exit_ctx = struct('zones', bridge_zones, ...
    'final_target_pose', wp_exit(end, :), ...
    'preplanned_route', true, ...
    'dynamic_pitch', USE_DYNAMIC_PITCH);
for i = 1:size(wp_exit, 1)
    wp = wp_exit(i, :);
    fprintf('    [bridge-pick] Exit %d/%d -> [%.1f, %.1f, %.1f, %.1f]\n', ...
        i, size(wp_exit, 1), wp(1), wp(2), wp(3), wp(4));
    hw.moveToPose(wp(1), wp(2), wp(3), wp(4), BRIDGE_MOVE_TIME, MODE_EXEC, Z_FLOOR, exit_ctx);
    pause(0.15);
end

end
