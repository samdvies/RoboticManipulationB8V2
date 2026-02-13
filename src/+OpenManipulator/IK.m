function joint_angles = IK(varargin)
%IK Inverse Kinematics for OpenManipulator-X
%   joint_angles = IK(x, y, z, pitch)
%   joint_angles = IK(T_ee)  (Not fully supported for pitch extraction yet, prefers x,y,z,pitch)
%
%   Input:
%       x: Target X (mm) - Forward
%       y: Target Y (mm) - Left (Internal/RH)
%       z: Target Z (mm) - Up
%       pitch: Target Pitch (degrees) - Relative to horizon
%              (0 = Horizontal, -90 = Pointing Down)
%
%   Output:
%       joint_angles: 1x4 vector [q1, q2, q3, q4] in DEGREES
%
%   Note: This function now uses the Internal (Right-Handed) Coordinate System:
%         X-Forward, Y-Left, Z-Up.
%         Selects "Elbow Down" configuration.

    if nargin == 1 && isequal(size(varargin{1}), [4 4])
        T = varargin{1};
        x_user = T(1, 4);
        y_user = T(2, 4);
        z_user = T(3, 4);
        % Extract pitch from rotation matrix?
        % For 4-DOF, orientation is restricted. We assume pitch is requested.
        % Simple extraction assuming planar final frame:
        % Pitch = atan2(R31, R11) or similar?
        % Let's require explicit pitch for robustness if possible. For now,
        % error if pitch not given, or assume vertical?
        error('Please provide (x, y, z, pitch) for robust 4-DOF IK.');
    elseif nargin == 4
        x_user = double(varargin{1});
        y_user = double(varargin{2});
        z_user = double(varargin{3});
        pitch_deg = double(varargin{4});
    else
        error('Invalid arguments. Usage: IK(x, y, z, pitch)');
    end

    % --- Coordinate Mapping ---
    % Used to map User(LH) to Internal(RH).
    % NOW UNIFIED: Input is already in Internal(RH) coordinates.
    % X: Forward, Y: Left, Z: Up.
    
    x = x_user;
    y = y_user;
    z = z_user;

    % DH Parameters (Lengths)
    dh = OpenManipulator.GetDH();
    d1 = dh(1, 3); % 77 (Base height)
    L2 = dh(3, 1); % 128 (Link 2 length - Shoulder to Elbow)
    L3 = dh(4, 1); % 124 (Link 3 length - Elbow to Wrist)
    L4 = dh(5, 1); % 126 (Link 4 length - Wrist to EE)

    % --- Step 1: Base Angle (q1) ---
    % atan2(y, x) in internal frame
    q1 = rad2deg(atan2(y, x));

    % --- Step 2: Planar Projection (Plane of the arm) ---
    % Distance in operations plane
    r = sqrt(x^2 + y^2);
    
    % Note: OpenManipulator has small offsets in DH, but primary links are centered?
    % DH: a1=0 (Link 1). d1=77. So projection is clean centered at 0.

    % Wrist Center (WC) Calculation
    % We want to place the End Effector (EE) at (r, z) with pitch `pitch_deg`.
    % Vector from WC to EE has length L4 and angle `pitch`.
    % r_ee = r_wc + L4 * cos(pitch)
    % z_ee = z_wc + L4 * sin(pitch)
    % So:
    pitch_rad = deg2rad(pitch_deg);
    r_wc = r - L4 * cos(pitch_rad);
    z_wc = (z - d1) - L4 * sin(pitch_rad); % Shift Z by base height d1

    % Distance from Shoulder (Joint 2) to Wrist Center
    D = sqrt(r_wc^2 + z_wc^2);

    % Check reachability
    max_reach = L2 + L3;
    if D > max_reach
        warning('Target out of reach. Clamping to max extent.');
        ratio = max_reach / D;
        r_wc = r_wc * ratio;
        z_wc = z_wc * ratio;
        D = max_reach; % Numerical stability
    end

    % --- Step 3: Elbow Angle (q3) ---
    % Law of Cosines on triangle (L2, L3, D)
    % D^2 = L2^2 + L3^2 - 2*L2*L3*cos(alpha)
    % cos(alpha) = (L2^2 + L3^2 - D^2) / (2*L2*L3)

    cos_alpha = (L2^2 + L3^2 - D^2) / (2*L2*L3);

    % Clamp for numerical error
    cos_alpha = max(min(cos_alpha, 1.0), -1.0);

    alpha = acos(cos_alpha); % Interior angle of triangle (always 0 to pi)
    
    % q3 definition in Modified DH for Reference.md params:
    % Link 3 theta offset is +90.
    % Let's verify q3 zero position.
    % If q3=0 (in FK input): theta = 0 + 90 = 90.
    % GetTransform(a=128, alpha=0, d=0, theta=90).
    % Rot_z(90) -> X points along Y.
    % Link 3 is along X usually.
    % Wait. The DH parameters describe the relationship.
    % Standard "Zero" pose for OpenManipulator: L-shape?
    % Usually, q3=0 means straight arm or 90 deg?
    % Let's assume the geometric alpha applies to the "internal" triangle angle.
    % Angle `q3_geom` is deviation from straight line.
    % alpha is interior angle. `q3_geom = pi - alpha`.
    %
    % "Elbow Down" vs "Elbow Up":
    % Elbow Down usually means the elbow 'dips'.
    % In our frame (Z up):
    % Elbow Up: Joint 3 is high (Triangle points up). q3 > 0?
    % Elbow Down: Joint 3 is low (Triangle points down). q3 < 0?
    % We have two solutions for alpha relative to line connecting shoulder/wrist.
    % But q3 is the relative angle.
    % Let's solve for the two configurations using beta (angle of D).
    
    % --- Step 4: Shoulder Angle (q2) ---
    % Angle of vector D (to WC) relative to horizontal:
    beta = atan2(z_wc, r_wc);

    % Angle of L2 relative to D (psi):
    % L3^2 = L2^2 + D^2 - 2*L2*D*cos(psi)
    % cos(psi) = (L2^2 + D^2 - L3^2) / (2*L2*D)
    cos_psi = (L2^2 + D^2 - L3^2) / (2*L2*D);
    cos_psi = max(min(cos_psi, 1.0), -1.0);
    psi = acos(cos_psi);

    % Solution 1 (Elbow Up/Standard):
    % q2 moves up from beta by psi.
    % q2_geom = beta + psi
    % q3_geom_triangle = (pi - alpha) (Shoulder to Elbow to Wrist bends)

    % Solution 2 (Elbow Down):
    % q2 moves down from beta by psi.
    % q2_geom = beta - psi
    % q3_geom_triangle = -(pi - alpha) (Bends the other way)

    % User Preference: Elbow Down.
    % This usually corresponds to Solution 2 (q2 lower).
    % Let's Select Solution 2.
    q2_geom = beta - psi;
    q3_geom = -(pi - alpha); % Negative bend? Or just the other sign?
    % Let's verify:
    % Loop closure: q2 + q3 + q_wrist = pitch?
    
    % Mapping Geometric Angles to DH Joint Angles [q2, q3]
    % Must account for DH offsets.
    % DH Link 2: theta = q2 - 90.
    % Geometry: usually measures q2 from horizontal X.
    % If DH q2=0 -> theta=-90 -> Points Down (-Y in previous frame, -Z in world?).
    % Wait. Link 1 Z is Up (Joint 1 axis). Link 1 X is in direction of arm.
    % Link 1 frame: X is forward. Z is Up.
    % Link 2 Joint axis is Y (Rotated by -90 alpha from Link 1 Z?).
    % DH Link 2 (Joint 2): alpha1=-90.
    % Rot_x(-90): Y -> Z, Z -> -Y.
    % So Frame 1 Z (axis of J2) is along previous -Y (Right?).
    % This gets confusing without visualization.
    %
    % Calibration Strategy:
    % Assume specific mapping offset `q_off`.
    % q2_output = rad2deg(q2_geom) + offset2
    % q3_output = rad2deg(q3_geom) + offset3
    %
    % Common OpenManipulator Offsets (from existing ROS packages/OpenCR):
    % q2: 0 is Up/Vertical? Or 0 is Horizontal?
    % Reference: "Home Position ... all zeros? L-shape?"
    % Usually q2=0 is vertical (up), q3=0 is horizontal (L-shape)?
    % If q2_geom (from horizontal) is 90 (Up), we want q2=0?
    % Then offset ~ -90.
    % If q2_geom is 0 (Horizontal), we want q2=90?
    % Let's simplify:
    % q2_dh = q2_geom - (offset_angle).
    %
    % Let's rely on the verified DH parameters in FK.
    % If we assume `GetDH` is correct.
    % TestIK will verify.
    % I will start with standard geometric mapping:
    % q2 = rad2deg(q2_geom) - 90? (To make vertical 0)
    % q3 = rad2deg(q3_geom) + 90?
    %
    % REVISION: The User wants "Elbow Down".
    % I will supply the code with "Elbow Down" logic (Solution 2).
    % AND I will tune the offsets in `TestIK` if `FK(IK)` fails.
    % I will inialize simple pass-through now and fix in Verification step.
    %
    % Based on DH:
    % T12 (J2): q2 - 90.
    % If target is horizontal (beta=0), q2_geom=0.
    % If arm is straight horizontal, q2-90 should be?
    % Link 2 is physical.
    % If theta=0, Frame 2 X aligns with Frame 1 X.
    % If theta=-90 (q2=0), Frame 2 X rotates -90.
    % This implies q2=0 makes the link point Up (Vertical).
    % So q2_geom (angle from horizontal) = q2_output + 90?
    % So q2_output = q2_geom - 90?
    % Wait. If q2_geom=90 (Up), q2_output=0. Correct.
    % So `q2 = rad2deg(q2_geom) - 90`. (Or +? let's try -90). NOTE: q2_geom is rad.
    
    % For q3:
    % T23 (J3): q3 + 90.
    % If q3=0, theta=90.
    % Frame 3 X rotates 90 relative to Frame 2 X.
    % If q2,q3=0 (L-shape). Link 2 Up. Link 3 Fwd?
    % q2=0 -> Link 2 Up. Frame 2 X Up.
    % q3=0 -> theta=90. Frame 3 X rotates 90 from Frame 2 X.
    % Vector along Link 3?
    % If a=128 (Link 2 length). Link 2 is along X2?
    % DH says a(i-1) is distance along X(i-1).
    % So Link 2 is along X1? No.
    % Modified DH: a_{i-1} is dist along X_{i-1}.
    % So Link 2 length `128` is along Frame 1 X (after q1 rotation)? No, Frame 2 X (after transform)?
    % In Modified DH, link length is associated with the *previous* frame?
    % `GetTransform(a, alpha, d, theta)`
    % `Trans_x(a)` is first.
    % So transformation from i-1 to i starts with translating along X_{i-1} by a_{i-1}.
    % So Link(i-1) length is along X_{i-1}.
    %
    % OK:
    % T01 (d=77). Z translation.
    % T12 (a=0). J2 is at same position as J1 frame origin?
    % T23 (a=128). Link 2 length.
    % T23 translates along X2 by 128.
    % So Link 2 is aligned with X2.
    % X2 depends on T12.
    % T12(q2-90).
    % If q2=0 (theta=-90). alpha=-90.
    % `Rot_x(-90) * Trans_x(0) * Rot_z(-90) * Trans_z(0)`
    % Standard Z1 (Up). Rot_x(-90) -> Z becomes Fwd (Y), Y becomes Up?
    % Then Rot_z(-90) (about new Z/Fwd) -> X (Right) becomes Up (-Y)?
    % This is hard to visualize mentally.
    %
    % Safe geometric formulas for OpenManipulator (from papers/code):
    % q1 = atan2(y, x)
    % q2 = atan2(z, r) ... + corrections
    %
    % I will Implement simple geometric logic with offsets derived from DH in `FK`.
    % Let's guess offsets: `q2_off = -90` (Vertical reference), `q3_off = 90`?
    %
    % Logic:
    % q2 = rad2deg(q2_geom) - (-79.38); % Tune this? No, standard is 90 or 0.
    % Let's use `q2 = rad2deg(q2_geom) - 90`.
    % And `q3 = rad2deg(q3_geom) + 90`.
    
    % --- Step 5: Wrist Angle (q4) ---
    % q234 = pitch.
    % q2_geom + q3_geom + q4_geom = pitch_rad.
    % q4_geom = pitch_rad - q2_geom - q3_geom.
    % q4_output = rad2deg(q4_geom).
    % Does q4 have offset?
    % DH Link 4 offset = 0.
    % So q4 = rad2deg(q4_geom).
    
    q2 = rad2deg(q2_geom) - 90.0; % Offset for L-shape home?
    q3 = rad2deg(q3_geom) + 90.0;
    q4 = rad2deg(pitch_rad - q2_geom - q3_geom);

    joint_angles = [q1, q2, q3, q4];

end
