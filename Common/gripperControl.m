function gripperControl(port_num, lib_name, action, config)
% GRIPPERCONTROL Control the OpenManipulator-X gripper
%
% Inputs:
%   port_num  - Dynamixel port handle
%   lib_name  - Dynamixel library name
%   action    - 'open', 'close', 'cube', or numeric encoder value
%   config    - Configuration struct (optional, uses defaults if not provided)
%
% Examples:
%   gripperControl(port_num, lib_name, 'open')
%   gripperControl(port_num, lib_name, 'cube', config)
%   gripperControl(port_num, lib_name, 400)  % Direct encoder value
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

%% Default Configuration
if nargin < 4
    config = struct();
    config.gripper.id = 15;
    config.gripper.open = 600;
    config.gripper.closed = 200;
    config.gripper.cube_grip = 400;  % 25mm cube
end

%% Dynamixel Constants
PROTOCOL_VERSION = 2.0;
ADDR_GOAL_POSITION = 116;
GRIPPER_ID = config.gripper.id;

%% Determine Target Position
if ischar(action) || isstring(action)
    switch lower(action)
        case 'open'
            target = config.gripper.open;
            action_name = 'OPEN';
        case 'close'
            target = config.gripper.closed;
            action_name = 'CLOSE';
        case 'cube'
            target = config.gripper.cube_grip;
            action_name = 'CUBE GRIP';
        otherwise
            error('Unknown gripper action: %s. Use open, close, cube, or a number.', action);
    end
elseif isnumeric(action)
    target = round(action);
    action_name = sprintf('POSITION %d', target);
else
    error('Invalid action type. Use string or numeric.');
end

%% Safety Check
if target < 0 || target > 1000
    error('Gripper position %d out of safe range [0-1000]', target);
end

%% Send Command
fprintf('Gripper: %s (encoder=%d)\n', action_name, target);
write4ByteTxRx(port_num, PROTOCOL_VERSION, GRIPPER_ID, ADDR_GOAL_POSITION, target);

%% Wait for Completion
pause(0.5);  % Allow gripper to move

end
