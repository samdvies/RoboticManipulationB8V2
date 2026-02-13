function [port_num, lib_name, cleanup] = robotSafeInit(com_port)
% ROBOTSAFEINIT Initializes Dynamixel SDK with safe velocity and position limits
%
% Sets up the OpenManipulator-X with:
%   - Profile Velocity = 50 (≈11.5 RPM, safe slow speed)
%   - Position Limits = [1024, 3072] (±90° from home)
%   - Automatic emergency stop on Ctrl+C via onCleanup
%
% DYNAMIXEL ANGLE CONVENTION:
%   Encoder 1024 = 90° (Wizard) = -90° from home
%   Encoder 2048 = 180° (Wizard) = 0° (home)
%   Encoder 3072 = 270° (Wizard) = +90° from home
%
% Usage:
%   [port_num, lib_name, cleanup] = robotSafeInit('COM3')
%
% Inputs:
%   com_port - Serial port name (e.g., 'COM3', 'COM4')
%
% Outputs:
%   port_num - Port handler for Dynamixel SDK
%   lib_name - Library name for calllib functions
%   cleanup  - onCleanup object (keep in scope to maintain auto-stop)
%
% IMPORTANT: Keep 'cleanup' variable in your workspace! If it goes out of
%            scope, the emergency stop will trigger automatically.
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Configuration Constants
    BAUDRATE = 1000000;
    PROTOCOL_VERSION = 2.0;
    
    % Motor IDs
    DXL_IDS = [11, 12, 13, 14, 15];  % Joints 1-4 + Gripper
    
    % Control Table Addresses (XM430-W350-T)
    ADDR_OPERATING_MODE     = 11;
    ADDR_TORQUE_ENABLE      = 64;
    ADDR_PROFILE_VELOCITY   = 112;
    ADDR_MIN_POSITION_LIMIT = 52;
    ADDR_MAX_POSITION_LIMIT = 48;
    
    % Safety Settings
    POSITION_CONTROL_MODE = 3;  % Operating mode for position control
    SAFE_VELOCITY = 50;        % ≈11.5 RPM (0.229 RPM per unit)
    
    % Per-joint position limits (with collision safety buffer)
    % Format: [min, max] in encoder units (2048 = home/0°)
    JOINT_LIMITS = [
        1024, 3072;   % Joint 1 (Base): ±90° - full range OK
        1024, 3072;   % Joint 2 (Shoulder): ±90° - full range OK  
        1024, 2900;   % Joint 3 (Elbow): -90° to +75° - avoid wrist-shoulder collision
        1200, 2900;   % Joint 4 (Wrist): -75° to +75° - avoid hitting base
    ];
    % Gripper (ID 15) uses default limits
    
    %% Load Dynamixel SDK Library
    lib_name = '';
    if strcmp(computer, 'PCWIN64')
        lib_name = 'dxl_x64_c';
    elseif strcmp(computer, 'GLNXA64')
        lib_name = 'libdxl_x64_c';
    elseif strcmp(computer, 'MACI64')
        lib_name = 'libdxl_mac_c';
    end
    
    if isempty(lib_name)
        error('Unsupported platform: %s', computer);
    end
    
    if ~libisloaded(lib_name)
        fprintf('Loading Dynamixel SDK library: %s\n', lib_name);
        [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
            'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
        
        if ~isempty(notfound)
            warning('Some functions not found: %s', strjoin(notfound, ', '));
        end
    else
        fprintf('Dynamixel SDK library already loaded\n');
    end
    
    %% Initialize Port
    fprintf('Opening port: %s at %d baud\n', com_port, BAUDRATE);
    
    port_num = portHandler(com_port);
    packetHandler();
    
    if ~openPort(port_num)
        error('Failed to open port %s. Check connection and COM port number.', com_port);
    end
    
    if ~setBaudRate(port_num, BAUDRATE)
        error('Failed to set baud rate to %d', BAUDRATE);
    end
    
    fprintf('Port opened successfully!\n');
    
    %% Setup Automatic Emergency Stop
    cleanup = onCleanup(@() cleanupFunction(port_num, lib_name));
    fprintf('Emergency stop configured (Ctrl+C will disable all motors)\n');
    
    %% Configure Each Motor
    fprintf('\n--- Configuring Motors with Safe Settings ---\n');
    fprintf('Profile Velocity: %d (≈%.1f RPM)\n', SAFE_VELOCITY, SAFE_VELOCITY * 0.229);
    fprintf('Per-joint limits (collision-safe):\n');
    joint_names = {'Base', 'Shoulder', 'Elbow', 'Wrist'};
    for j = 1:4
        min_deg = (JOINT_LIMITS(j,1) - 2048) * 360 / 4096;
        max_deg = (JOINT_LIMITS(j,2) - 2048) * 360 / 4096;
        fprintf('  %s: [%.0f°, %.0f°]\n', joint_names{j}, min_deg, max_deg);
    end
    fprintf('----------------------------------------------\n\n');
    
    for i = 1:length(DXL_IDS)
        id = DXL_IDS(i);
        
        % First, ensure torque is disabled (required to change settings)
        write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 0);
        pause(0.05);
        
        % Set operating mode to Position Control (CRITICAL!)
        write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE);
        pause(0.05);
        
        % Set position limits (per-joint for IDs 11-14, default for gripper)
        if id >= 11 && id <= 14
            joint_idx = id - 10;
            min_pos = JOINT_LIMITS(joint_idx, 1);
            max_pos = JOINT_LIMITS(joint_idx, 2);
        else
            % Gripper - use wider limits
            min_pos = 0;
            max_pos = 4095;
        end
        write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MIN_POSITION_LIMIT, min_pos);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_MAX_POSITION_LIMIT, max_pos);
        
        % Set profile velocity (safe slow speed)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PROFILE_VELOCITY, SAFE_VELOCITY);
        
        % Check for communication errors
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= 0
            warning('Motor ID %d: Communication error (code %d)', id, dxl_comm_result);
        else
            fprintf('Motor ID %d: Configured successfully\n', id);
        end
    end
    
    fprintf('\n=== Initialization Complete ===\n');
    fprintf('Motors are configured but torque is DISABLED.\n');
    fprintf('Use enableMotor() to enable individual motors.\n');
    fprintf('Press Ctrl+C at any time for emergency stop.\n\n');
end

%% Cleanup Function (called automatically on Ctrl+C or error)
function cleanupFunction(port_num, lib_name)
    fprintf('\n>>> Cleanup triggered - Disabling all motors...\n');
    
    DXL_IDS = [11, 12, 13, 14, 15];
    ADDR_TORQUE_ENABLE = 64;
    PROTOCOL_VERSION = 2.0;
    
    for id = DXL_IDS
        try
            write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 0);
        catch
            % Ignore errors during cleanup
        end
    end
    
    % Close port
    try
        closePort(port_num);
        fprintf('>>> Port closed\n');
    catch
        % Ignore
    end
    
    fprintf('>>> All motors disabled - Safe to handle robot\n');
end
