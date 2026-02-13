function emergencyStop(port_num, lib_name)
% EMERGENCYSTOP Immediately disables torque on all OpenManipulator-X motors
%
% This function should be called when an emergency stop is needed.
% It disables torque on all 5 motors (4 arm joints + gripper).
%
% USAGE:
%   emergencyStop(port_num, lib_name)
%
% Also use with onCleanup for automatic stop on Ctrl+C:
%   cleanupObj = onCleanup(@() emergencyStop(port_num, lib_name));
%
% Inputs:
%   port_num - Port handler number from Dynamixel SDK
%   lib_name - Name of loaded Dynamixel library (e.g., 'dxl_x64_c')
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Motor IDs and Control Table
    DXL_IDS = [11, 12, 13, 14, 15];  % Joint 1-4 + Gripper
    ADDR_TORQUE_ENABLE = 64;
    PROTOCOL_VERSION = 2.0;
    
    %% Disable Torque on All Motors
    fprintf('\n!!! EMERGENCY STOP ACTIVATED !!!\n');
    
    for i = 1:length(DXL_IDS)
        id = DXL_IDS(i);
        try
            % Write torque disable command
            calllib(lib_name, 'write1ByteTxRx', port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 0);
            fprintf('Motor ID %d: Torque DISABLED\n', id);
        catch ME
            fprintf('Motor ID %d: Failed to disable - %s\n', id, ME.message);
        end
    end
    
    fprintf('!!! ALL MOTORS STOPPED !!!\n\n');
end
