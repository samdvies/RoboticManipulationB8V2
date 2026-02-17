% TEST_GRIPPER Demonstrates and verifies Gripper control
%
% Usage:
%   1. Connect OpenManipulator-X
%   2. Run this script
%
% Expectation:
%   - Connects to robot
%   - Opens gripper fully
%   - Closes gripper fully
%   - Moves to 50%
%   - Reads and prints positions

clc; clear;
addpath(genpath('../src')); % Ensure src is available

try
    fprintf('=== Gripper Test Start ===\n');
    
    % 1. Create Interface
    % Adjust COM port if necessary
    port = 'COM4'; 
    baud = 1000000;
    fprintf('Connecting to %s at %d baud...\n', port, baud);
    
    hw = OpenManipulator.HardwareInterface(port, baud);
    
    % 2. Configure (CRITICAL: Sets Operating Mode)
    fprintf('Configuring...\n');
    hw.configure(20);

    % 3. Enable Torque
    hw.enableTorque();
    
    % 4. Test Open
    fprintf('\nTesting OPEN...\n');
    hw.openGripper();
    pause(5);
    pos = hw.readGripperPosition();
    fprintf('Position: %.1f%%\n', pos);
    
    % 4. Test Close
    fprintf('\nTesting CLOSE...\n');
    hw.closeGripper();
    pause(5);
    pos = hw.readGripperPosition();
    fprintf('Position: %.1f%%\n', pos);
    
    % 5. Test Partial (50%)
    fprintf('\nTesting 50%%...\n');
    hw.setGripperPosition(50);
    pause(5);
    pos = hw.readGripperPosition();
    fprintf('Position: %.1f%%\n', pos);
    
    % 6. Clean up
    fprintf('\nTest Complete. Disconnecting.\n');
    hw.disconnect();
    
catch ME
    fprintf('\nERROR: %s\n', ME.message);
    if exist('hw', 'var')
        hw.disconnect();
    end
end
