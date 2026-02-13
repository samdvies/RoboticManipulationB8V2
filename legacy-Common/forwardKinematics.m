function [T_all, pos_endeff, R_endeff] = forwardKinematics(q)
% FORWARDKINEMATICS Computes forward kinematics for OpenManipulator-X
%
% Returns all intermediate transformation matrices for IK use and visualization.
%
% DYNAMIXEL ANGLE CONVENTION:
%   - q = 0 corresponds to encoder value 2048 (home position)
%   - Positive q = CCW rotation (increasing encoder value)
%   - Operating range: q ∈ [-π/2, +π/2] maps to encoder [1024, 3072]
%
% DH Parameters (Craig Convention) for OpenManipulator-X:
%   Frame | a(mm)   | alpha(rad) | d(mm) | theta
%   ------|---------|------------|-------|------------------
%     1   |   0     |     0      |  77   | q1
%     2   |   0     |   π/2      |   0   | q2 + β
%     3   | 130.23  |     0      |   0   | q3 - β
%     4   |  124    |     0      |   0   | q4
%    tool |  126    |     0      |   0   | 0
%
%   β = atan2(24, 128) ≈ 0.1855 rad (10.62°) - mechanical offset
%
% Usage:
%   [T_all, pos, R] = forwardKinematics([q1, q2, q3, q4])
%
% Inputs:
%   q - 1x4 vector of joint angles in radians [q1, q2, q3, q4]
%       q(1): Base rotation
%       q(2): Shoulder angle
%       q(3): Elbow angle
%       q(4): Wrist angle
%
% Outputs:
%   T_all      - Cell array {T_0_1, T_0_2, T_0_3, T_0_4, T_0_tool}
%                Each is a 4x4 homogeneous transform from base to that frame
%   pos_endeff - 3x1 vector [x; y; z] end-effector position in mm
%   R_endeff   - 3x3 rotation matrix of end-effector orientation
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Robot Physical Parameters (mm)
    L_base = 77;                              % Height from base to joint 2
    L_prox_x = 24;                            % Horizontal offset of link 2
    L_prox_z = 128;                           % Vertical component of link 2
    L_prox = sqrt(L_prox_x^2 + L_prox_z^2);   % Diagonal length ≈ 130.23 mm
    L_dist = 124;                             % Link 3 length (joint 3 to 4)
    L_tool = 126;                             % Tool length (joint 4 to end-effector)
    
    % Mechanical offset angle due to slanted link geometry
    beta = atan2(L_prox_x, L_prox_z);         % ≈ 0.1855 rad (10.62°)
    
    %% Extract Joint Angles
    q1 = q(1);  % Base rotation
    q2 = q(2);  % Shoulder
    q3 = q(3);  % Elbow
    q4 = q(4);  % Wrist
    
    %% Compute Individual Transformations (DH Convention)
    % T_i_j means transform from frame i to frame j
    
    % Base frame {0} to Joint 1 frame {1}
    T_0_1 = buildDHMatrix(0, 0, L_base, q1);
    
    % Joint 1 {1} to Joint 2 {2}
    T_1_2 = buildDHMatrix(0, pi/2, 0, q2 + beta);
    
    % Joint 2 {2} to Joint 3 {3}
    T_2_3 = buildDHMatrix(L_prox, 0, 0, q3 - beta);
    
    % Joint 3 {3} to Joint 4 {4}
    T_3_4 = buildDHMatrix(L_dist, 0, 0, q4);
    
    % Joint 4 {4} to Tool frame {tool}
    T_4_tool = buildDHMatrix(L_tool, 0, 0, 0);
    
    %% Chain Transformations (Cumulative from Base)
    T_0_2 = T_0_1 * T_1_2;
    T_0_3 = T_0_2 * T_2_3;
    T_0_4 = T_0_3 * T_3_4;
    T_0_tool = T_0_4 * T_4_tool;
    
    %% Package Outputs
    % Cell array of all transforms from base frame
    T_all = {T_0_1, T_0_2, T_0_3, T_0_4, T_0_tool};
    
    % End-effector position (4th column of T_0_tool)
    pos_endeff = T_0_tool(1:3, 4);
    
    % End-effector orientation (3x3 rotation matrix)
    R_endeff = T_0_tool(1:3, 1:3);
end
