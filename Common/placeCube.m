function placeCube(hw, target_xy, stack_level, is_rotated, pickup_angle_deg, cfg)
% PLACECUBE  Place a held cube at target, accounting for stack height.
% Default pitch -90 (downwards); pitch 0 (forwards) when cube was rotated.
%
% Inputs:
%   hw               - OpenManipulator.HardwareInterface instance
%   target_xy        - [x, y] target position (mm)
%   stack_level      - 0 = on surface, 1 = on top of 1 cube, 2 = on top of 2, etc.
%   is_rotated       - logical; true -> pitch 0 (forwards), false -> pitch -90 (downwards)
%   pickup_angle_deg - angle at which this cube was picked (0, 22.5, or 45); angle-based
%                      place offset is applied only when is_rotated is true
%   cfg              - config struct from task_config()

tx = target_xy(1);
ty = target_xy(2);

place_z = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE/2 + cfg.PICK_Z_OFFSET_MM + (stack_level * cfg.CUBE_SIZE);
% Use higher Z when moving to place so path clears the bridge
approach_z = cfg.PLACE_APPROACH_Z;

if is_rotated
    pitch = 0;
else
    pitch = -90;
end

% Place offset: magnitude toward origin + (if is_rotated) angle-specific offset by pick angle + tuning
r_place = sqrt(tx^2 + ty^2);
if r_place > 1e-6
    off_x = -cfg.PLACE_OFFSET_MAG * tx / r_place;
    off_y = -cfg.PLACE_OFFSET_MAG * ty / r_place;
else
    off_x = 0;
    off_y = 0;
end
if is_rotated
    if abs(pickup_angle_deg - 0) < 1
        po = cfg.PLACE_OFFSET_0;
    elseif abs(pickup_angle_deg - 22.5) < 1
        po = cfg.PLACE_OFFSET_22_5;
    elseif abs(pickup_angle_deg - 45) < 1
        po = cfg.PLACE_OFFSET_45;
    else
        po = [0, 0];
    end
else
    po = [0, 0];
end
act_x = tx + off_x + po(1) + cfg.place_offset_tuning(1);
act_y = ty + off_y + po(2) + cfg.place_offset_tuning(2);

fprintf('    [place] Target (%.1f, %.1f) stack=%d  pick_angle=%.1f°  offset=(%.2f, %.2f)  actual=(%.1f, %.1f) z=%.1f pitch=%.0f\n', ...
    tx, ty, stack_level, pickup_angle_deg, off_x + po(1) + cfg.place_offset_tuning(1), off_y + po(2) + cfg.place_offset_tuning(2), act_x, act_y, place_z, pitch);

fprintf('    [place] Above target (z=%.1f to clear bridge)...\n', approach_z);
hw.moveToPose(act_x, act_y, approach_z, pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

above_place_z = place_z + cfg.PLACE_VERTICAL_OFFSET_MM;
fprintf('    [place] Directly above target (z=%.1f, straight-down approach)...\n', above_place_z);
hw.moveToPose(act_x, act_y, above_place_z, pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

drop_z = place_z + cfg.PLACE_DROP_MM;
fprintf('    [place] Lower to z=%.1f (drop from 1 mm above rest)...\n', drop_z);
hw.moveToPose(act_x, act_y, drop_z, pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.5);

fprintf('    [place] Open gripper...\n');
hw.openGripper();
pause(1.0);

fprintf('    [place] Retract to approach height...\n');
hw.moveToPose(act_x, act_y, approach_z, pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

end
