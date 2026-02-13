function joint_angles = IK(x, y, z, pitch, method)
%IK Inverse Kinematics for OpenManipulator-X
%   joint_angles = IK(x, y, z, pitch)
%   joint_angles = IK(x, y, z, pitch, method)
%
%   Matches the verified Python IK implementation exactly.
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
    d1 = dh(1, 3);   % 77  (Base height)
    L2 = dh(3, 1);   % 128 (Shoulder to Elbow)
    L3 = dh(4, 1);   % 124 (Elbow to Wrist)
    L4 = dh(5, 1);   % 126 (Wrist to EE)

    % --- Step 1: Base Angle (q1) ---
    q1 = rad2deg(atan2(y, x));

    % --- Step 2: Planar Projection ---
    r = sqrt(x^2 + y^2);

    pitch_rad = deg2rad(pitch_deg);
    r_wc = r - L4 * cos(pitch_rad);
    z_wc = (z - d1) - L4 * sin(pitch_rad);

    % Distance from Shoulder to Wrist Center
    D = sqrt(r_wc^2 + z_wc^2);

    % Check reachability
    max_reach = L2 + L3;
    if D > max_reach
        warning('Target out of reach. Clamping to max extent.');
        ratio = max_reach / D;
        r_wc = r_wc * ratio;
        z_wc = z_wc * ratio;
        D = max_reach;
    end

    % --- Step 3: Elbow Angle ---
    cos_alpha = (L2^2 + L3^2 - D^2) / (2 * L2 * L3);
    cos_alpha = max(min(cos_alpha, 1.0), -1.0);
    alpha = acos(cos_alpha);

    % --- Step 4: Shoulder Angle ---
    beta = atan2(z_wc, r_wc);
    cos_psi = (L2^2 + D^2 - L3^2) / (2 * L2 * D);
    cos_psi = max(min(cos_psi, 1.0), -1.0);
    psi = acos(cos_psi);

    if strcmpi(method, 'elbow_up')
        % Solution 1: Elbow Up
        q2_geom = beta + psi;
        q3_geom = (pi - alpha);
    else
        % Solution 2: Elbow Down
        q2_geom = beta - psi;
        q3_geom = -(pi - alpha);
    end

    % --- Step 5: Map geometric angles to DH joint angles ---
    % These offsets match the verified Python IK:
    %   DH theta2 = q2 - 90  =>  q2 = 90 - q2_geom (axis inversion)
    %   DH theta3 = q3 + 90  =>  q3 = q3_geom - 90
    q2 = 90.0 - rad2deg(q2_geom);
    q3 = rad2deg(q3_geom) - 90.0;

    % --- Step 6: Wrist Angle (q4) ---
    % Inverted pitch coupling (matches Python):
    %   q4 = -pitch - q2 - q3
    q4 = -pitch_deg - q2 - q3;

    joint_angles = [q1, q2, q3, q4];

    % Enforce joint limits for safe operation
    [joint_angles, ~] = OpenManipulator.JointLimits.Clamp(joint_angles);
end
