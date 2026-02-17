% VERIFY_JACOBIAN_MATH
clc; clear;
addpath(genpath('../src'));

diary('verify_output.txt');
diary on;

fprintf('=== Jacobian Math Verification ===\n');

% 1. Define a known configuration (Home Pose, roughly L-shape)
% In degrees
q_home = [0, -45, 45, 0]; 

fprintf('Configuration: [%.1f, %.1f, %.1f, %.1f]\n', q_home);

% 2. Compute Jacobian
J = OpenManipulator.GetJacobian(q_home);

disp('Jacobian Matrix J:');
disp(J);

% 3. Check Dimensions
if ~isequal(size(J), [6, 4])
    error('Jacobian has incorrect dimensions: %dx%d (Expected 6x4)', size(J,1), size(J,2));
end
fprintf('Dimensions OK.\n');

% 4. Check Singularity (Condition Number)
c = cond(J(1:3, :)); % Condition number of linear velocity part
fprintf('Condition Number (Linear): %.4f\n', c);

if c > 100
    fprintf('[WARNING] Configuration is near singularity.\n');
else
    fprintf('Configuration is well-conditioned.\n');
end

% 5. Check Physical Consistency
% Joint 1 rotates around Z-axis. It should produce velocity in Y direction (purely lateral)
% if the arm is extended along X.
% At q1=0, arm is in X-Z plane.
% J(:, 1) should be [0; vy; 0; 0; 0; 1] (Linear Y, Angular Z)
col1 = J(:, 1);
fprintf('Joint 1 Column: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', col1);
% Check logic...

% Joint 2 (Shoulder): Rotates around Y-axis (Horizontal). Should produce X-Z motion.
% J(:, 2) should have vy ~ 0.
col2 = J(:, 2);
fprintf('Joint 2 Column: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', col2);

if abs(col2(2)) < 1e-3 && (abs(col2(1)) > 1e-3 || abs(col2(3)) > 1e-3)
    fprintf('Joint 2 logic checks out (rotates around Y, moves in X-Z).\n');
else
    fprintf('[WARNING] Joint 2 logic UNEXPECTED. Vy=%.2f (Should be 0)\n', col2(2));
end

diary off;

if abs(col1(1)) < 1e-6 && abs(col1(3)) < 1e-6 && abs(col1(6)) > 0.9
    fprintf('Joint 1 logic checks out (rotates around Z, moves in Y).\n');
else
    fprintf('[WARNING] Joint 1 logic seems unexpected. Check frames.\n');
end

fprintf('Verification Complete.\n');
