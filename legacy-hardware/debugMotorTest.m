function debugMotorTest(port_num, lib_name)
% DEBUGMOTORTEST Minimal motor test with verbose debug output
%
% Run after robotSafeInit to debug motor issues
%
% Usage:
%   [port_num, lib_name, cleanup] = robotSafeInit('COM3');
%   debugMotorTest(port_num, lib_name);

    PROTOCOL_VERSION = 2.0;
    
    % Addresses
    ADDR_TORQUE_ENABLE = 64;
    ADDR_GOAL_POSITION = 116;
    ADDR_PRESENT_POSITION = 132;
    ADDR_PROFILE_VELOCITY = 112;
    ADDR_OPERATING_MODE = 11;
    
    % Test just motor 11 (base)
    DXL_ID = 11;
    
    fprintf('\n=== DEBUG MOTOR TEST ===\n');
    fprintf('Testing Motor ID: %d\n\n', DXL_ID);
    
    %% Step 1: Read current position
    fprintf('Step 1: Reading current position...\n');
    pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    fprintf('  Position: %d (comm=%d, err=%d)\n', pos, comm_result, dxl_error);
    
    %% Step 2: Read operating mode
    fprintf('\nStep 2: Reading operating mode...\n');
    mode = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE);
    fprintf('  Operating Mode: %d (3=Position, 4=Extended Position)\n', mode);
    
    %% Step 3: Read profile velocity
    fprintf('\nStep 3: Reading profile velocity...\n');
    vel = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY);
    fprintf('  Profile Velocity: %d\n', vel);
    
    %% Step 4: Read torque status
    fprintf('\nStep 4: Reading torque enable status...\n');
    torque = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE);
    fprintf('  Torque Enable: %d (0=OFF, 1=ON)\n', torque);
    
    %% Step 5: Enable torque
    fprintf('\nStep 5: Enabling torque...\n');
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, 1);
    comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    fprintf('  Write result: comm=%d, err=%d\n', comm_result, dxl_error);
    pause(0.1);
    
    % Verify torque is enabled
    torque = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE);
    fprintf('  Torque Enable NOW: %d\n', torque);
    
    if torque ~= 1
        fprintf('\n*** TORQUE FAILED TO ENABLE! ***\n');
        fprintf('Check if motor is in correct mode.\n');
        return;
    end
    
    %% Step 6: Command movement
    fprintf('\nStep 6: Commanding movement...\n');
    current_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    fprintf('  Current position: %d\n', current_pos);
    
    % Move +200 encoder units (small movement)
    target_pos = current_pos + 200;
    if target_pos > 3072
        target_pos = current_pos - 200;  % Go other direction if at limit
    end
    
    fprintf('  Target position: %d\n', target_pos);
    fprintf('  Sending goal position command...\n');
    
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, target_pos);
    comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    fprintf('  Write result: comm=%d, err=%d\n', comm_result, dxl_error);
    
    %% Step 7: Monitor movement
    fprintf('\nStep 7: Monitoring movement (3 seconds)...\n');
    for i = 1:6
        pause(0.5);
        pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        fprintf('  t=%.1fs: position=%d (target=%d, diff=%d)\n', i*0.5, pos, target_pos, abs(pos-target_pos));
    end
    
    %% Step 8: Disable torque
    fprintf('\nStep 8: Disabling torque...\n');
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, 0);
    
    fprintf('\n=== DEBUG COMPLETE ===\n');
    fprintf('If position did not change, check:\n');
    fprintf('  1. Motor LED blinking? (indicates error)\n');
    fprintf('  2. Operating mode correct? (should be 3 or 4)\n');
    fprintf('  3. Hardware error in Dynamixel Wizard?\n');
    fprintf('  4. Position limits blocking movement?\n');
end
