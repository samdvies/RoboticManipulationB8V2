function debugFullMovement(port_num, lib_name)
% DEBUGFULLMOVEMENT Carefully debug multi-joint movement
%
% Tests each joint individually, then together, with full verification.
%
% Usage:
%   debugFullMovement(port_num, lib_name)

PROTOCOL_VERSION = 2.0;
DXL_IDS = [11, 12, 13, 14];
ADDR_TORQUE_ENABLE = 64;
ADDR_GOAL_POSITION = 116;
ADDR_PRESENT_POSITION = 132;
ADDR_PROFILE_VELOCITY = 112;
ADDR_MOVING = 122;  % 1 if motor is moving, 0 if stopped

fprintf('\n=== FULL MOVEMENT DEBUG ===\n\n');

%% Step 1: Read current state of all motors
fprintf('Step 1: Current motor state\n');
for id = DXL_IDS
    pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRESENT_POSITION);
    torque = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE);
    vel = read4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_VELOCITY);
    fprintf('  Motor %d: pos=%d, torque=%d, velocity=%d\n', id, pos, torque, vel);
end

%% Step 2: Enable torque on all and go to home ONE BY ONE
fprintf('\nStep 2: Moving to home position ONE MOTOR AT A TIME\n');
input('Press ENTER to start (watch robot carefully)...');

for i = 1:4
    id = DXL_IDS(i);
    fprintf('  Enabling and homing motor %d...\n', id);
    
    % Enable torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 1);
    pause(0.1);
    
    % Set slow velocity
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_VELOCITY, 15);
    pause(0.1);
    
    % Command home
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, 2048);
    
    % Wait until stopped
    waitForMotor(port_num, PROTOCOL_VERSION, id);
    
    pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRESENT_POSITION);
    fprintf('    Motor %d at: %d (target: 2048, error: %d)\n', id, pos, abs(pos-2048));
    
    pause(0.5);
end

fprintf('All motors at home.\n');

%% Step 3: Test a simple IK move - SEQUENTIAL motors
fprintf('\nStep 3: Test IK movement to [350, 0, 150] - SEQUENTIAL\n');
target = [350, 0, 150];
[q, success, info] = inverseKinematicsAuto(target);

if ~success
    fprintf('IK failed: %s\n', info.message);
    return;
end

encoders = zeros(1, 4);
for i = 1:4
    encoders(i) = angleConversion('rad2enc', q(i));
end
fprintf('Target encoders: [%d, %d, %d, %d]\n', encoders);

input('Press ENTER to move SEQUENTIALLY...');

% Move each motor one at a time, wait for completion
for i = 1:4
    id = DXL_IDS(i);
    target_enc = encoders(i);
    
    fprintf('  Moving motor %d to %d...\n', id, target_enc);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, target_enc);
    
    waitForMotor(port_num, PROTOCOL_VERSION, id);
    
    pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRESENT_POSITION);
    fprintf('    Motor %d at: %d (error: %d)\n', id, pos, abs(pos-target_enc));
    
    pause(0.3);
end

% Check final position
fprintf('\n  Final position check:\n');
final_enc = zeros(1, 4);
for i = 1:4
    final_enc(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDS(i), ADDR_PRESENT_POSITION);
    fprintf('    Motor %d: target=%d, actual=%d, error=%d\n', ...
        DXL_IDS(i), encoders(i), final_enc(i), abs(encoders(i)-final_enc(i)));
end

% Compute FK
final_q = zeros(1, 4);
for i = 1:4
    final_q(i) = angleConversion('enc2rad', final_enc(i));
end
[~, actual_pos, ~] = forwardKinematics(final_q);
fprintf('\n  Target XYZ: [%.1f, %.1f, %.1f]\n', target);
fprintf('  Actual XYZ: [%.1f, %.1f, %.1f]\n', actual_pos);
fprintf('  Position error: %.1f mm\n', norm(actual_pos' - target));

%% Step 4: Return home sequential
fprintf('\nStep 4: Returning home SEQUENTIALLY\n');
input('Press ENTER...');

for i = 1:4
    id = DXL_IDS(i);
    fprintf('  Homing motor %d...\n', id);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, 2048);
    waitForMotor(port_num, PROTOCOL_VERSION, id);
end

fprintf('\n=== DEBUG COMPLETE ===\n');

end

%% Helper: Wait for motor to stop moving
function waitForMotor(port_num, protocol, id)
    ADDR_MOVING = 122;
    ADDR_PRESENT_POSITION = 132;
    ADDR_GOAL_POSITION = 116;
    
    timeout = 10;  % seconds
    start = tic;
    
    while toc(start) < timeout
        moving = read1ByteTxRx(port_num, protocol, id, ADDR_MOVING);
        if moving == 0
            % Double-check position is stable
            pause(0.1);
            moving2 = read1ByteTxRx(port_num, protocol, id, ADDR_MOVING);
            if moving2 == 0
                return;
            end
        end
        pause(0.05);
    end
    fprintf('    WARNING: Motor %d timeout!\n', id);
end
