%% check_zero_pose.m
% Simple script to move the arm to [0.0, 0.0, 0.0, 0.0] degrees
% so you can physically verify the geometry against the FK target.
%
% Usage:
%   run('scripts/check_zero_pose.m')

clc; clear;
addpath(genpath('../src'));

PORT = 'COM4';
BAUD = 1000000;
VELOCITY = 20;

% Target [0,0,0,0]
angles_deg = [0, 0, 0, 0];

fprintf('==========================================\n');
fprintf('  Checking Physical Zero Pose: [0, 0, 0, 0]\n');
fprintf('==========================================\n');

% Theoretical FK
[T, ~] = OpenManipulator.FK(angles_deg);
theoretical_ee = T(1:3, 4)';
fprintf('Theoretical FK End-Effector: [%.1f, %.1f, %.1f] mm\n', ...
    theoretical_ee(1), theoretical_ee(2), theoretical_ee(3));
fprintf('------------------------------------------\n');
fprintf('Expected Physical State:\n');
fprintf('  - Base facing exactly forward\n');
fprintf('  - Shoulder leaning slightly forward (~10.6 deg)\n');
fprintf('  - Elbow and Wrist forming a perfectly horizontal line extending forward\n');
fprintf('  - Measure Z-height from table to center of closed gripper: should be %.1f mm\n', theoretical_ee(3));
fprintf('  - Measure X-length from base rotation axis to center of closed gripper: should be %.1f mm\n', theoretical_ee(1));
fprintf('==========================================\n\n');

input('Press ENTER to connect and move arm to [0, 0, 0, 0]...', 's');

try
    hw = OpenManipulator.HardwareInterface(PORT, BAUD);
    hw.configure(VELOCITY);
    hw.enableTorque();
    
    % Close gripper so you can measure the physical center exactly
    hw.closeGripper();
    pause(1);

    fprintf('Moving slowly to [0, 0, 0, 0]...\n');
    % Move taking 4 seconds so it's smooth
    hw.moveToAnglesInterpolated(angles_deg, 40);
    
    fprintf('\nArm is at [0, 0, 0, 0]. You can measure it now.\n');
    input('Press ENTER when done measuring to release torque and disconnect...', 's');
    
    hw.disableTorque();
    hw.disconnect();
    fprintf('Done.\n');
    
catch ME
    fprintf('\nERROR: %s\n', ME.message);
    try hw.disableTorque(); hw.disconnect(); catch, end
end
