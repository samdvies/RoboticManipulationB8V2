function dh_params = GetDH()
%GETDH Returns the Modified DH parameters for OpenManipulator-X
%   dh_params = [a(i-1), alpha(i-1), d(i), theta_offset(i)]
%   Units: mm, degrees
%
%   Matches Reference.md and Python DH.py (straight-link model).
%
%   Home pose (q=[0,0,0,0]):
%     - Shoulder link points straight up (128mm vertical)
%     - Elbow + Wrist links extend forward (124+126 = 250mm horizontal)
%     - EE position: [250, 0, 205]

    %           a(i-1)  alpha(i-1)  d(i)    theta_offset
    dh_params = [0,      0,          77,     0;      % Joint 1 (Base)
                 0,     -90,         0,     -90;     % Joint 2 (Shoulder)
                 128,    0,          0,      90;     % Joint 3 (Elbow)
                 124,    0,          0,      0;      % Joint 4 (Wrist)
                 126,    0,          0,      0];     % End Effector
end
