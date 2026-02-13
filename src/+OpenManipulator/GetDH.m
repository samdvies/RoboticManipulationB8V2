function dh_params = GetDH()
%GETDH Returns the Modified DH parameters for OpenManipulator-X
%   dh_params = [a(i-1), alpha(i-1), d(i), theta_offset(i)]
%   Units: mm, degrees
%
%   Rows correspond to Link 1, 2, 3, 4.
%   Note: theta_offset is added to the joint angle input q.

    %           a(i-1)  alpha(i-1)  d(i)    theta_offset
    dh_params = [0,      0,          77,     0;          % Link 1
                 0,     -90,         0,     -90;         % Link 2
                 128,    0,          0,      90;         % Link 3
                 124,    0,          0,      0;          % Link 4
                 126,    0,          0,      0];         % End Effector (Fixed link from Joint 4)
    
    % Note: The Reference.md lists 4 links + End Effector.
    % Link 1 (Joint 1 to 2): a=0, alpha=0, d=77, theta=q1
    % Link 2 (Joint 2 to 3): a=0, alpha=-90, d=0, theta=q2-90
    % Link 3 (Joint 3 to 4): a=128, alpha=0, d=0, theta=q3+90
    % Link 4 (Joint 4 to EE): a=124, alpha=0, d=0, theta=q4
    % Wait, the Reference table row 4 says a_{i-1}=124. This connects Link 3 to Link 4.
    % Row 5 (End): a_{i-1}=126. This connects Link 4 to End Effector.
    
    % Let's refine the matrix to match the transformation chain T01, T12, T23, T34, T4E
    % T01: a0=0,   alpha0=0,   d1=77, q1
    % T12: a1=0,   alpha1=-90, d2=0,  q2-90
    % T23: a2=128, alpha2=0,   d3=0,  q3+90
    % T34: a3=124, alpha3=0,   d4=0,  q4
    % T4E: a4=126, alpha4=0,   dE=0,  0 (Fixed)
    
    dh_params = [0,      0,      77,     0;    % Joint 1
                 0,     -90,     0,     -90;   % Joint 2
                 128,    0,      0,      90;   % Joint 3
                 124,    0,      0,      0;    % Joint 4
                 126,    0,      0,      0];   % End Effector
end
