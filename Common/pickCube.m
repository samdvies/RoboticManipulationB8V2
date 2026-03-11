function pickCube(hw, cube_xy, for_rotation, pickup_angle_deg, cfg, cube_index)
% PICKCUBE  Standard pick: approach, descend, grip, lift.
% Transit moves use TRANSIT_PITCH (e.g. -45) so the arm stays within reach
% at high Z.  Pitch -90 is used only at low Z for the actual grip.
%
% Inputs:
%   hw              - OpenManipulator.HardwareInterface instance
%   cube_xy         - [x, y] position of cube (mm)
%   for_rotation    - logical (kept for compatibility; offset is by angle)
%   pickup_angle_deg - inferred pick bearing (0, 22.5, or 45); selects offset when per-cube not used
%   cfg             - config struct from task_config()
%   cube_index     - (optional) when provided and cfg.cube_pick_offsets exists, use that row for [radial, tangential]

if nargin < 6, cube_index = []; end

pick_x = cube_xy(1);
pick_y = cube_xy(2);

% Per-cube pick offset (Task 2a) overrides angle-based when present
use_per_cube = ~isempty(cube_index) && isfield(cfg, 'cube_pick_offsets') && ...
    cube_index >= 1 && cube_index <= size(cfg.cube_pick_offsets, 1);

if use_per_cube
    off = cfg.cube_pick_offsets(cube_index, :);
elseif for_rotation
    if isfield(cfg, 'PICK_OFFSET_0')
        if abs(pickup_angle_deg - 0) < 1
            off = cfg.PICK_OFFSET_0;
        elseif abs(pickup_angle_deg - 22.5) < 1
            off = cfg.PICK_OFFSET_22_5;
        elseif abs(pickup_angle_deg - 45) < 1
            off = cfg.PICK_OFFSET_45;
        else
            off = [0, 0];
        end
    else
        off = [0, 0];
    end
else
    if abs(pickup_angle_deg - 0) < 1
        off = cfg.PICK_OFFSET_STD_0;
    elseif abs(pickup_angle_deg - 22.5) < 1
        off = cfg.PICK_OFFSET_STD_22_5;
    elseif abs(pickup_angle_deg - 45) < 1
        off = cfg.PICK_OFFSET_STD_45;
    else
        off = [0, 0];
    end
end
bearing = atan2(pick_y, pick_x);
pick_x = pick_x + off(1)*cos(bearing) - off(2)*sin(bearing);
pick_y = pick_y + off(1)*sin(bearing) + off(2)*cos(bearing);

pick_z  = cfg.CUBE_Z_SURFACE + cfg.CUBE_SIZE / 2 + cfg.PICK_Z_OFFSET_MM;
hover_z = cfg.HOVER_Z;
transit_pitch = cfg.TRANSIT_PITCH;
travel_z = cfg.PLACE_APPROACH_Z;

fprintf('    [pick] Above cube (%.1f, %.1f) at z=%.1f (pitch %.0f transit)...\n', pick_x, pick_y, hover_z, transit_pitch);
hw.moveToPose(pick_x, pick_y, hover_z, transit_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

fprintf('    [pick] Descend to cube z=%.1f (pitch -90)...\n', pick_z);
hw.moveToPose(pick_x, pick_y, pick_z, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

fprintf('    [pick] Lock pitch at -90...\n');
hw.moveToPose(pick_x, pick_y, pick_z, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

fprintf('    [pick] Close gripper...\n');
hw.closeGripper();
pause(cfg.PICK_HOLD_TIME);

fprintf('    [pick] Lift %d mm...\n', cfg.PICK_LIFT_MM);
hw.moveToPose(pick_x, pick_y, pick_z + cfg.PICK_LIFT_MM, -90, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.15);

fprintf('    [pick] Lift to travel height z=%.1f (pitch %.0f)...\n', travel_z, transit_pitch);
hw.moveToPose(pick_x, pick_y, travel_z, transit_pitch, cfg.MOVE_TIME, cfg.MOTION_MODE, cfg.Z_FLOOR);
pause(0.25);

end
