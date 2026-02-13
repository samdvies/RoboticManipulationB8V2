function T = buildDHMatrix(a, alpha, d, theta)
% BUILDDHMATRIX Constructs a 4x4 homogeneous transformation matrix using
% Craig's DH convention (Modified DH Parameters).
%
% DYNAMIXEL ANGLE CONVENTION:
%   - Encoder 2048 = Home position (180° in Wizard)
%   - Increasing encoder values = CCW rotation (viewing motor from shaft)
%   - This matches standard DH: positive theta = CCW about Z-axis
%
% DH Parameters (Craig Convention):
%   a     : Link length (distance along X_{i} from Z_{i-1} to Z_{i})
%   alpha : Link twist (angle about X_{i} from Z_{i-1} to Z_{i})
%   d     : Link offset (distance along Z_{i-1} from X_{i-1} to X_{i})
%   theta : Joint angle (angle about Z_{i-1} from X_{i-1} to X_{i})
%
% Transformation order: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
%
% Usage:
%   T = buildDHMatrix(a, alpha, d, theta)
%
% Inputs:
%   a     - Link length in mm
%   alpha - Link twist in radians
%   d     - Link offset in mm
%   theta - Joint angle in radians
%
% Output:
%   T     - 4x4 homogeneous transformation matrix
%
% Example:
%   T01 = buildDHMatrix(0, 0, 77, q1);  % Base to Joint 1
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    % Precompute trigonometric values for efficiency
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    
    % Build transformation matrix (Craig DH Convention)
    % T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    T = [ct,    -st,     0,     a;
         st*ca,  ct*ca, -sa,   -d*sa;
         st*sa,  ct*sa,  ca,    d*ca;
         0,      0,      0,     1];
end
