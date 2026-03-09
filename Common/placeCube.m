function placeCube(hw, target_xy, stack_level, is_rotated, cfg)
% PLACECUBE  Place a held cube at target, accounting for stack height.
% Default pitch -90 (downwards); pitch 0 (forwards) when cube was rotated.
%
% Inputs:
%   hw          - OpenManipulator.HardwareInterface instance
%   target_xy   - [x, y] target position (mm)
%   stack_level - 0 = on surface, 1 = on top of 1 cube, 2 = on top of 2, etc.
%   is_rotated  - logical; true -> pitch 0 (forwards), false -> pitch -90 (downwards)
%   cfg         - config struct from task_config()

tx = target_xy(1);
ty = target_xy(2);

place_z = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE/2 + (stack_level * cfg.CUBE_SIZE);
% Use higher Z when moving to place so path clears the bridge
approach_z = cfg.PLACE_APPROACH_Z;

if is_rotated
    pitch = 0;
else
    pitch = -90;
end

% Place offset: magnitude toward origin (from existing offset model)
r_place = sqrt(tx^2 + ty^2);
if r_place > 1e-6
    off_x = -cfg.PLACE_OFFSET_MAG * tx / r_place;
    off_y = -cfg.PLACE_OFFSET_MAG * ty / r_place;
else
    off_x = 0;
    off_y = 0;
end

% Apply automatic offset + user tuning offset
act_x = tx + off_x + cfg.place_offset_tuning(1);
act_y = ty + off_y + cfg.place_offset_tuning(2);

fprintf('    [place] Target (%.1f, %.1f) stack=%d  offset=(%.2f, %.2f)  actual=(%.1f, %.1f) z=%.1f pitch=%.0f\n', ...
    tx, ty, stack_level, off_x + cfg.place_offset_tuning(1), off_y + cfg.place_offset_tuning(2), act_x, act_y, place_z, pitch);

fprintf('    [place] Above target (z=%.1f to clear bridge)...\n', approach_z);
hw.moveToPose(act_x, act_y, approach_z, pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

fprintf('    [place] Lower to z=%.1f...\n', place_z);
hw.moveToPose(act_x, act_y, place_z, pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.5);

fprintf('    [place] Open gripper...\n');
hw.openGripper();
pause(1.0);

fprintf('    [place] Retract to approach height...\n');
hw.moveToPose(act_x, act_y, approach_z, pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

end
