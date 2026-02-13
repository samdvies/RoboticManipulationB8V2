function T = GetTransform(a, alpha, d, theta)
%GETTRANSFORM Computes the homogeneous transform for Modified DH parameters
%   T = GetTransform(a, alpha, d, theta)
%
%   Modified DH (Craig Convention):
%     T(i-1,i) = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
%
%   Inputs (scalars):
%       a:     Link length (mm)
%       alpha: Link twist (degrees)
%       d:     Link offset (mm)
%       theta: Joint angle (degrees)
%
%   Output:
%       T: 4x4 homogeneous transformation matrix

    % Convert degrees to radians
    alphar = deg2rad(alpha);
    thetar = deg2rad(theta);

    ct = cos(thetar);
    st = sin(thetar);
    ca = cos(alphar);
    sa = sin(alphar);

    % Modified DH Transformation Matrix (Craig Convention)
    % T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
    T = [ct,      -st,      0,    a;
         st*ca,    ct*ca,  -sa,  -d*sa;
         st*sa,    ct*sa,   ca,   d*ca;
         0,        0,       0,    1];
end
