function placeCube(hw, target_xy, stack_level, is_rotated, pickup_angle_deg, cfg, release_z_adjust)
% PLACECUBE  Place a held cube at target, accounting for stack height.
% Rotated places use pitch 0 throughout.
% Standard places use TRANSIT_PITCH for approach/retract (reachable at high Z)
% and pitch -90 only at low Z for the actual drop.
%
% Inputs:
%   hw               - OpenManipulator.HardwareInterface instance
%   target_xy        - [x, y] target position (mm)
%   stack_level      - 0 = on surface, 1 = on top of 1 cube, 2 = on top of 2, etc.
%   is_rotated       - logical; true -> pitch 0 (forwards), false -> pitch -90 (downwards)
%   pickup_angle_deg - PICK angle (inferred pick bearing 0, 22.5, or 45); selects
%                      which place offset set to use (same logic as rotational offsets).
%   cfg              - config struct from task_config()
%   release_z_adjust - (optional) mm to add to release height; negative = lower (default 0)

if nargin < 7, release_z_adjust = 0; end

tx = target_xy(1);
ty = target_xy(2);

place_z = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE/2 + cfg.PICK_Z_OFFSET_MM + (stack_level * cfg.CUBE_SIZE);
approach_z = cfg.PLACE_APPROACH_Z;

if is_rotated
    work_pitch    = 0;
    transit_pitch = 0;
else
    work_pitch    = -90;
    transit_pitch = cfg.TRANSIT_PITCH;
end

if is_rotated
    r_place = sqrt(tx^2 + ty^2);
    if r_place > 1e-6
        off_x = -cfg.PLACE_OFFSET_MAG * tx / r_place;
        off_y = -cfg.PLACE_OFFSET_MAG * ty / r_place;
    else
        off_x = 0;
        off_y = 0;
    end
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
    off_x = 0;
    off_y = 0;
    if abs(pickup_angle_deg - 0) < 1
        po = cfg.PLACE_OFFSET_STD_0;
    elseif abs(pickup_angle_deg - 22.5) < 1
        po = cfg.PLACE_OFFSET_STD_22_5;
    elseif abs(pickup_angle_deg - 45) < 1
        po = cfg.PLACE_OFFSET_STD_45;
    else
        po = [0, 0];
    end
end
bearing_place = atan2(ty, tx);
po_wx = po(1)*cos(bearing_place) - po(2)*sin(bearing_place);
po_wy = po(1)*sin(bearing_place) + po(2)*cos(bearing_place);
act_x = tx + off_x + po_wx + cfg.place_offset_tuning(1);
act_y = ty + off_y + po_wy + cfg.place_offset_tuning(2);

fprintf('    [place] Target (%.1f, %.1f) stack=%d  pick_angle=%.1f°  offset=(%.2f, %.2f)  actual=(%.1f, %.1f) z=%.1f work_pitch=%.0f transit_pitch=%.0f\n', ...
    tx, ty, stack_level, pickup_angle_deg, off_x + po_wx + cfg.place_offset_tuning(1), off_y + po_wy + cfg.place_offset_tuning(2), act_x, act_y, place_z, work_pitch, transit_pitch);

fprintf('    [place] Above target (z=%.1f, pitch %.0f)...\n', approach_z, transit_pitch);
hw.moveToPose(act_x, act_y, approach_z, transit_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

above_place_z = place_z + cfg.PLACE_VERTICAL_OFFSET_MM;
fprintf('    [place] Directly above target (z=%.1f, pitch %.0f)...\n', above_place_z, work_pitch);
hw.moveToPose(act_x, act_y, above_place_z, work_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

drop_z = place_z + cfg.PLACE_DROP_MM + release_z_adjust;
fprintf('    [place] Lower to z=%.1f (drop height, adjust=%.1f mm)...\n', drop_z, release_z_adjust);
hw.moveToPose(act_x, act_y, drop_z, work_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

fprintf('    [place] Lock pitch at %.0f...\n', work_pitch);
hw.moveToPose(act_x, act_y, drop_z, work_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

fprintf('    [place] Open gripper...\n');
hw.openGripper();
pause(0.5);

fprintf('    [place] Retract (z=%.1f, pitch %.0f)...\n', approach_z, transit_pitch);
hw.moveToPose(act_x, act_y, approach_z, transit_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

end
