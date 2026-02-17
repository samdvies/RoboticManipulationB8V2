% DEBUG_GRIPPER Diagnostic script for Gripper (ID 15)
%
% Usage:
%   run('scripts/debug_gripper.m')

clc; clear;
addpath(genpath('../src'));

PORT = 'COM4';
BAUD = 1000000;
ID = 15;
PROTOCOL = 2.0;

% Addresses (XM430-W350-T)
ADDR_OPERATING_MODE = 11;
ADDR_TORQUE_ENABLE  = 64;
ADDR_LED            = 65;
ADDR_HARDWARE_ERROR = 70;
ADDR_GOAL_POSITION  = 116;
ADDR_PRESENT_POSITION = 132;
ADDR_PRESENT_CURRENT  = 126;

% Load SDK
lib_name = '';
if strcmp(computer, 'PCWIN64'), lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNXA64'), lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64'), lib_name = 'libdxl_mac_c';
end

if ~libisloaded(lib_name)
    [~, ~] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', ...
        'addheader', 'packet_handler.h');
end

% Open Port
port_num = portHandler(PORT);
packetHandler();

if openPort(port_num)
    fprintf('Port Open: OK\n');
else
    error('Failed to open port');
end

if setBaudRate(port_num, BAUD)
    fprintf('Baud Rate: OK\n');
else
    error('Failed to set baud rate');
end

fprintf('\n--- DIAGNOSTICS FOR ID %d ---\n', ID);

% 1. Ping
model_number = pingGetModelNum(port_num, PROTOCOL, ID);
if getLastTxRxResult(port_num, PROTOCOL) == 0
    fprintf('Ping: OK (Model %d)\n', model_number);
else
    fprintf('Ping: FAILED (Comm Error)\n');
end

% 2. Check Hardware Error
hw_err = read1ByteTxRx(port_num, PROTOCOL, ID, ADDR_HARDWARE_ERROR);
fprintf('Hardware Error Status: %d\n', hw_err);

% 3. Check Operating Mode
op_mode = read1ByteTxRx(port_num, PROTOCOL, ID, ADDR_OPERATING_MODE);
fprintf('Operating Mode: %d (Expected: 3 for Position)\n', op_mode);

% 4. Check Torque Enable
torque = read1ByteTxRx(port_num, PROTOCOL, ID, ADDR_TORQUE_ENABLE);
fprintf('Torque Enable: %d\n', torque);

% 5. Check Present Position
pos = read4ByteTxRx(port_num, PROTOCOL, ID, ADDR_PRESENT_POSITION);
fprintf('Present Position: %d\n', pos);

% 6. LED Test (Visual Check)
fprintf('\nToggle LED test...\n');
write1ByteTxRx(port_num, PROTOCOL, ID, ADDR_LED, 1);
pause(1);
write1ByteTxRx(port_num, PROTOCOL, ID, ADDR_LED, 0);
fprintf('LED toggled.\n');

% Close
closePort(port_num);
fprintf('Disconnected.\n');
