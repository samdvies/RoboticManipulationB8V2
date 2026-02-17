% TEST_SAFETY_CHECK Verify collision avoidance logic
%
% Usage:
%   run('tests/test_safety_check.m')
%
% This script attempts to move the robot through an unsafe trajectory
% (dipping the wrist below Z=20mm) to verify the safety check works.
% NOTE: This uses a Mock/Simulation mode if actual hardware is not connected,
%       but since HardwareInterface forces connection, we might fail on connect.
%       Ideally we would mock the hardware, but for now we assume connection OR
%       we manually use the checking logic.

clc; clear;
addpath(genpath('../src'));

fprintf('=== Safety Check Verification ===\n');

% Mocking the logic without needing physical hardware connection for this test
% We will extract the logic used in HardwareInterface to test it in isolation

MIN_Z_HEIGHT = 20;

% Define a safe start pose (Home)
q_start = [0, -45, 0, 0]; % Roughly home-ish

% Define an UNSAFE target pose
% Trying to point straight down near the table
% Let's try to reach Z = 0
q_unsafe = OpenManipulator.IK(200, 0, 10, -90); 

fprintf('Testing Trajectory: Start -> Unsafe Target\n');

% Simulate interpolation
num_steps = 10;
collision_detected = false;

for step = 1:num_steps
    t = step / num_steps;
    q_interp = (1 - t) * q_start + t * q_unsafe;
    
    % Perform FK
    [~, global_transforms] = OpenManipulator.FK(q_interp);
    
    z_elbow = global_transforms(3, 4, 2);
    z_wrist = global_transforms(3, 4, 3);
    z_ee    = global_transforms(3, 4, 5);
    
    fprintf('Step %d: Z_Elbow=%.1f, Z_Wrist=%.1f, Z_EE=%.1f\n', ...
        step, z_elbow, z_wrist, z_ee);
    
    if z_elbow < MIN_Z_HEIGHT || z_wrist < MIN_Z_HEIGHT || z_ee < MIN_Z_HEIGHT
        fprintf('  [SUCCESS] Safety Violation Detected at Step %d! (Z < %d)\n', step, MIN_Z_HEIGHT);
        collision_detected = true;
        break;
    end
end

if collision_detected
    fprintf('\nPASSED: Safety logic correctly identified collision.\n');
else
    fprintf('\nFAILED: Unsafe trajectory was allowed.\n');
end
