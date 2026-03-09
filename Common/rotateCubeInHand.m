function rotateCubeInHand(hw, current_xy, cfg)
% ROTATECUBEINHAND  Rotate a held cube from vertical (-90) to horizontal (0).
%
% If the cube is too close to the robot base (r < MIN_ROTATE_RADIUS), the
% arm pulls out radially to a safe radius first, rotates, then returns.
% Rotation is always pitch -90 -> 0 (toward robot), matching ROTATE_MODE=1
% from the existing pick-rotate scripts.
%
% Inputs:
%   hw         - OpenManipulator.HardwareInterface instance
%   current_xy - [x, y] position where the cube is currently held (mm)
%   cfg        - config struct from task_config()

cx = current_xy(1);
cy = current_xy(2);
hover_z = cfg.HOVER_Z;

r_cube = sqrt(cx^2 + cy^2);
needs_pullout = (r_cube < cfg.MIN_ROTATE_RADIUS) && (r_cube > 0.001);

if needs_pullout
    scale  = cfg.MIN_ROTATE_RADIUS / r_cube;
    safe_x = cx * scale;
    safe_y = cy * scale;
    fprintf('    [rotate] Pull out to safe radius (%.1f, %.1f)...\n', safe_x, safe_y);
    hw.moveToPose(safe_x, safe_y, hover_z, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
    pause(0.3);
    rot_x = safe_x;
    rot_y = safe_y;
else
    rot_x = cx;
    rot_y = cy;
end

fprintf('    [rotate] Pitch -90 -> 0...\n');
hw.moveToPose(rot_x, rot_y, hover_z, 0, cfg.MOVE_TIME * 1.5, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.5);

if needs_pullout
    fprintf('    [rotate] Return to (%.1f, %.1f)...\n', cx, cy);
    hw.moveToPose(cx, cy, hover_z, 0, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
    pause(0.3);
end

final_x = cx; final_y = cy;
fprintf('    [rotate] Lift to travel height z=%.1f (clear bridge)...\n', cfg.PLACE_APPROACH_Z);
hw.moveToPose(final_x, final_y, cfg.PLACE_APPROACH_Z, 0, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.3);

end
