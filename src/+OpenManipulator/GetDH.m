function dh_params = GetDH()
%GETDH Returns the Modified DH parameters for OpenManipulator-X
%   dh_params = [a(i-1), alpha(i-1), d(i), theta_offset(i)]
%   Units: mm, degrees
%
%   Slanted-link model: accounts for the physical 24mm forward offset
%   in the shoulder-to-elbow link (Joint 2 to Joint 3).
%
%   Physical geometry:
%     Link 2-3: 128mm vertical + 24mm forward => L_prox = 130.23mm
%     Tilt angle: beta = atan2(24, 128) = 10.62 degrees
%
%   Home pose (q=[0,0,0,0]):
%     - Shoulder link: mostly up, tilted 10.6 deg forward
%     - Joint 3 at [24, 0, 205] (24mm ahead of shoulder axis)
%     - Elbow + Wrist: horizontal forward (124+126 = 250mm)
%     - EE position: [274, 0, 205]

    L_prox_x = 24;   % Forward offset (mm)
    L_prox_z = 128;  % Vertical component (mm)
    L_prox = sqrt(L_prox_x^2 + L_prox_z^2);        % 130.23 mm
    beta_deg = atan2d(L_prox_x, L_prox_z);           % 10.62 deg

    %           a(i-1)  alpha(i-1)  d(i)    theta_offset
    dh_params = [0,      0,          77,     0;                 % Joint 1 (Base)
                 0,     -90,         0,     -90 + beta_deg;     % Joint 2 (Shoulder)
                 L_prox,  0,         0,      90 - beta_deg;     % Joint 3 (Elbow)
                 124,    0,          0,      0;                 % Joint 4 (Wrist)
                 126,    0,          0,      0];                % End Effector
end

