
%% HardwareSyncController.m
% Polls the Bridge Server for new targets and moves the OpenManipulator-X.
%
% This script is designed for debugging joint rotation directions by 
% providing a "ground truth" simulation for manual comparison.

clear all;
close all;
clc;

% Configuration COM_PORT = 'COM3';
% Adjust to your port POLL_INTERVAL = 0.5;
% seconds BRIDGE_URL = 'http://localhost:3001/command';
BRIDGE_STATE_URL = 'http://localhost:3001/state';

% Initialize Hardware try[port_num, lib_name, cleanup] =
    robotSafeInit(COM_PORT);
enableMotor(port_num, lib_name, [ 11, 12, 13, 14 ]);
fprintf('Hardware initialized successfully.\n');
catch e warning('Hardware initialization failed: %s', e.message);
fprintf('Running in simulation-only mode (Bridge polling active).\n');
port_num = -1;
end

    % Keep track of last processed command to avoid jitter last_command =
    struct('x', 0, 'y', 0, 'z', 0, 'pitch', 0);

fprintf('Starting sync loop. Control+C to stop.\n');

while true
    try
        % 1. Poll Bridge for Latest Command
        options = weboptions('Timeout', 1);
cmd = webread(BRIDGE_URL, options);

if
  ~isempty(fieldnames(cmd)) &&
      ~isequal(cmd, last_command)
          fprintf('\nNew target received: X=%.0f, Y=%.0f, Z=%.0f, Pitch=%.0f\n',
                  ... cmd.x, cmd.y, cmd.z, cmd.pitch);

% 2. Calculate Theoretical IK target_pos = [cmd.x; cmd.y; cmd.z];
            % Using existing project IK function
            [q_target, success, info] = inverseKinematicsAuto(target_pos');
            
            if success
                fprintf('  IK Success. Joint angles (deg): [%.1f, %.1f, %.1f, %.1f]\n', ...
                    rad2deg(q_target(1)), rad2deg(q_target(2)), ...
                    rad2deg(q_target(3)), rad2deg(q_target(4)));
                
                % 3. Move Hardware (Very Slowly)
                if port_num ~= -1
                    fprintf('  Moving hardware...\n');
                    % Use 15 (very slow) for Profile Velocity override if needed
                    % setProfileVelocity(port_num, lib_name, [11,12,13,14], 15);
                    
                    % Bulk move to target
                    % Note: verify motor directions by comparing to simulation!
                    moveJoint(port_num, lib_name, [11, 12, 13, 14], q_target);
                end
            else
                fprintf('  IK Failed: Target unreachable according to MATLAB model.\n');
            end
            
            last_command = cmd;
        end
        
    catch e
        if contains(e.message, 'Connection refused')
            fprintf('Waiting for Bridge Server...\n');
        else
            fprintf('Error in loop: %s\n', e.message);
        end
    end
    
    pause(POLL_INTERVAL);
end
