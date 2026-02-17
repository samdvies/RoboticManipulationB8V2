% SIM_JACOBIAN_MOTION
% Offline simulation of Jacobian-based control verification.
% Generates a trajectory plot to verify straight-line motion.

clc; clear; close all;
addpath(genpath('../src'));

fprintf('=== Jacobian Simulation (No Hardware) ===\n');

% Configuration
DT = 0.05;
LAMBDA = 0.01;
DURATION = 4.0; 
steps = DURATION / DT;

% Initial State (Home-ish)
% Start at X=200, Y=0, Z=100
q = OpenManipulator.IK(200, 0, 100, -90);
history_pos = zeros(steps, 3);
history_vel = zeros(steps, 3);

fprintf('Simulating Linear Move in Y-axis (Side-to-Side)...\n');
fprintf('Target Velocity: Sinusoidal Y motion.\n');

t = 0;
for i = 1:steps
    % 1. FK to get current position
    T = OpenManipulator.FK(q);
    p_curr = T(1:3, 4);
    history_pos(i, :) = p_curr';
    
    % 2. Get Jacobian
    J = OpenManipulator.GetJacobian(q);
    
    % 3. Define Target Spatial Velocity
    % Move purely in Y direction: +/- 40 mm/s
    vy = 40 * sin(2 * pi * t / 2.0); 
    vx = 0;
    vz = 0;
    
    v_target = [vx; vy; vz; 0; 0; 0];
    history_vel(i, :) = [vx, vy, vz];
    
    % 4. Damped Least Squares Inverse
    lambda_sq = LAMBDA^2;
    J_dls = J' * inv(J * J' + lambda_sq * eye(6));
    q_dot_rad = J_dls * v_target;
    
    % 5. Integrate
    q_dot_deg = rad2deg(q_dot_rad);
    q = q + q_dot_deg' * DT;
    
    % 6. Check Limits
    [q, ~] = OpenManipulator.JointLimits.Clamp(q);
    
    t = t + DT;
end

% Check linearity
% For a pure Y move, X and Z should be constant.
x_std = std(history_pos(:, 1));
z_std = std(history_pos(:, 3));

fprintf('Simulation Complete.\n');
fprintf('Path Deviation (Std Dev):\n');
fprintf('  X: %.4f mm (Should be ~0)\n', x_std);
fprintf('  Z: %.4f mm (Should be ~0)\n', z_std);

% Visualize
figure('Visible', 'off'); % Don't pop up window, save it.
subplot(2,1,1);
plot(history_pos(:, 2), history_pos(:, 1), 'LineWidth', 2);
xlabel('Y (mm)'); ylabel('X (mm)');
title('Top View (X-Y)');
grid on; axis equal;

subplot(2,1,2);
plot(history_pos(:, 2), history_pos(:, 3), 'LineWidth', 2);
xlabel('Y (mm)'); ylabel('Z (mm)');
title('Front View (Z-Y)');
grid on; axis equal;

saveas(gcf, 'sim_trajectory_plot.png');
fprintf('Trajectory plot saved to sim_trajectory_plot.png\n');
