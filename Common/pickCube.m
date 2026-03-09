function pickCube(hw, cube_xy, for_rotation, cfg)
% PICKCUBE  Standard pick: approach, descend, grip, lift.
% Uses pitch -90 (downwards) as default for vertical grip.
%
% Inputs:
%   hw            - OpenManipulator.HardwareInterface instance
%   cube_xy       - [x, y] position of cube (mm)
%   for_rotation  - logical; if true, adds PICK_OFFSET_X to X for grip alignment
%   cfg           - config struct from task_config()

pick_x = cube_xy(1);
pick_y = cube_xy(2);
if for_rotation
    pick_x = pick_x + cfg.PICK_OFFSET_X;
end

pick_z  = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2;
hover_z = cfg.HOVER_Z;

fprintf('    [pick] Above cube (%.1f, %.1f) at z=%.1f (pitch -90 straight down)...\n', pick_x, pick_y, hover_z);
hw.moveToPose(pick_x, pick_y, hover_z, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

fprintf('    [pick] Descend to cube z=%.1f...\n', pick_z);
hw.moveToPose(pick_x, pick_y, pick_z, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.5);

fprintf('    [pick] Close gripper...\n');
hw.closeGripper();
pause(cfg.PICK_HOLD_TIME);

fprintf('    [pick] Lift %d mm...\n', cfg.PICK_LIFT_MM);
hw.moveToPose(pick_x, pick_y, pick_z + cfg.PICK_LIFT_MM, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

fprintf('    [pick] Lift to travel height z=%.1f (clear bridge)...\n', cfg.PLACE_APPROACH_Z);
hw.moveToPose(pick_x, pick_y, cfg.PLACE_APPROACH_Z, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.5);

end
