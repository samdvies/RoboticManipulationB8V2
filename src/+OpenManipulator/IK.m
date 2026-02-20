function joint_angles = IK(x, y, z, pitch, method)
%IK Inverse Kinematics for OpenManipulator-X
%   joint_angles = IK(x, y, z, pitch)
%   joint_angles = IK(x, y, z, pitch, method)
%
%   Slanted-link IK accounting for the 24mm forward offset in the
%   shoulder-to-elbow link (L_prox = 130.23mm, beta = 10.62 deg).
%
%   Input:
%       x:      Target X (mm) - Forward
%       y:      Target Y (mm) - Left (Internal RH frame)
%       z:      Target Z (mm) - Up
%       pitch:  Target Pitch (degrees) - 0 = Horizontal, -90 = Down
%       method: 'elbow_up' (default) or 'elbow_down'
%
%   Output:
%       joint_angles: 1x4 vector [q1, q2, q3, q4] in DEGREES
%
%   Coordinate System: X-Forward, Y-Left, Z-Up (Right-Handed)

    if nargin < 5
        method = 'elbow_up';
    end

    x = double(x);
    y = double(y);
    z = double(z);
    pitch_deg = double(pitch);

    % DH Parameters (Lengths in mm)
    dh = OpenManipulator.GetDH();
    d1      = dh(1, 3);   % 77    (Base height)
    L_prox  = dh(3, 1);   % 130.23 (Shoulder to Elbow, slanted)
    L_dist  = dh(4, 1);   % 124   (Elbow to Wrist)
    L_tool  = dh(5, 1);   % 126   (Wrist to EE)

    % Beta: tilt angle of the shoulder link
    beta_deg = atan2d(24, 128);  % 10.62 degrees

    % --- Step 1: Base Angle (q1) ---
    q1 = rad2deg(atan2(y, x));

    % --- Step 2: Planar Projection ---
    r = sqrt(x^2 + y^2);

    pitch_rad = deg2rad(pitch_deg);
    r_wc = r - L_tool * cos(pitch_rad);
    z_wc = (z - d1) - L_tool * sin(pitch_rad);

    % Distance from Shoulder to Wrist Center
    D = sqrt(r_wc^2 + z_wc^2);

    % Check reachability
    max_reach = L_prox + L_dist;
    if D > max_reach
        warning('Target out of reach. Clamping to max extent.');
        ratio = max_reach / D;
        r_wc = r_wc * ratio;
        z_wc = z_wc * ratio;
        D = max_reach;
    end

    % --- Step 3: Elbow Angle ---
    cos_alpha = (L_prox^2 + L_dist^2 - D^2) / (2 * L_prox * L_dist);
    cos_alpha = max(min(cos_alpha, 1.0), -1.0);
    alpha = acos(cos_alpha);

    % --- Step 4: Shoulder Angle ---
    beta_r = atan2(z_wc, r_wc);
    cos_psi = (L_prox^2 + D^2 - L_dist^2) / (2 * L_prox * D);
    cos_psi = max(min(cos_psi, 1.0), -1.0);
    psi = acos(cos_psi);

    if strcmpi(method, 'elbow_up')
        q2_geom = beta_r + psi;
        q3_geom = (pi - alpha);
    else
        q2_geom = beta_r - psi;
        q3_geom = -(pi - alpha);
    end

    % --- Step 5: Map geometric angles to DH joint angles ---
    % With slanted-link theta offsets (-90+beta) and (90-beta):
    %   q2 = (90 - beta) - q2_geom_deg
    %   q3 = q3_geom_deg - (90 - beta)
    q2 = (90.0 - beta_deg) - rad2deg(q2_geom);
    q3 = rad2deg(q3_geom) - (90.0 - beta_deg);

    % --- Step 6: Wrist Angle (q4) ---
    % Pitch coupling: pitch = -(q2 + q3 + q4)
    q4 = -pitch_deg - q2 - q3;

    joint_angles = [q1, q2, q3, q4];

    % Enforce joint limits for safe operation
    [joint_angles, ~] = OpenManipulator.JointLimits.Clamp(joint_angles);
end

