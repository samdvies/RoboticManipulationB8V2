function placeBridgeRotationStaging(hw, target_xy, pickup_angle_deg, cfg)
% PLACEBRIDGEROTATIONSTAGING  Stage a bridge-picked cube for the standard rotate flow.
%
% Keeps the validated bridge-pick hold at pitch 0 through placement, opens
% the gripper, retracts vertically to a safe height, and only then changes
% pitch so the existing pickCube -> rotateCubeInHand path can be reused.
%
% Inputs:
%   hw               - OpenManipulator.HardwareInterface instance
%   target_xy        - [x, y] staging holder position (mm)
%   pickup_angle_deg - original pick bearing bucket (0, 22.5, or 45)
%   cfg              - config struct from task_config()

tx = target_xy(1);
ty = target_xy(2);

place_z = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE/2 + cfg.PICK_Z_OFFSET_MM;
approach_z = cfg.PLACE_APPROACH_Z;
work_pitch = 0;
post_release_pitch = cfg.TRANSIT_PITCH;

if abs(pickup_angle_deg - 0) < 1
    po = cfg.PLACE_OFFSET_STD_0;
elseif abs(pickup_angle_deg - 22.5) < 1
    po = cfg.PLACE_OFFSET_STD_22_5;
elseif abs(pickup_angle_deg - 45) < 1
    po = cfg.PLACE_OFFSET_STD_45;
else
    po = [0, 0];
end

bearing_place = atan2(ty, tx);
po_wx = po(1) * cos(bearing_place) - po(2) * sin(bearing_place);
po_wy = po(1) * sin(bearing_place) + po(2) * cos(bearing_place);

act_x = tx + po_wx + cfg.place_offset_tuning(1);
act_y = ty + po_wy + cfg.place_offset_tuning(2);

fprintf('    [bridge-stage] Target (%.1f, %.1f) pick_angle=%.1f deg actual=(%.1f, %.1f) z=%.1f\n', ...
    tx, ty, pickup_angle_deg, act_x, act_y, place_z);

fprintf('    [bridge-stage] Above target (z=%.1f, pitch %.0f)...\n', approach_z, work_pitch);
hw.moveToPose(act_x, act_y, approach_z, work_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

above_place_z = place_z + cfg.PLACE_VERTICAL_OFFSET_MM;
fprintf('    [bridge-stage] Directly above target (z=%.1f, pitch %.0f)...\n', above_place_z, work_pitch);
hw.moveToPose(act_x, act_y, above_place_z, work_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

drop_z = place_z + cfg.PLACE_DROP_MM;
fprintf('    [bridge-stage] Lower to z=%.1f (pitch %.0f)...\n', drop_z, work_pitch);
hw.moveToPose(act_x, act_y, drop_z, work_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

fprintf('    [bridge-stage] Open gripper...\n');
hw.openGripper();
pause(0.5);

fprintf('    [bridge-stage] Retract to z=%.1f at pitch %.0f...\n', approach_z, work_pitch);
hw.moveToPose(act_x, act_y, approach_z, work_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

fprintf('    [bridge-stage] Change pitch to %.0f for standard re-pick...\n', post_release_pitch);
hw.moveToPose(act_x, act_y, approach_z, post_release_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

end
