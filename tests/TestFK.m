% TestFK.m - Script
addpath('src');

try
    fprintf('Testing Forward Kinematics...\n');

    % Home Position
    q = [0, 0, 0, 0];
    [T, ~] = OpenManipulator.FK(q);
    disp('Home Pose T:');
    disp(T);

    fprintf('Testing Inverse Kinematics...\n');
    % User Target: X=200, Y=50, Z=100, Pitch=0
    % NOTE: Y=50 means LEFT in Internal/RH coordinates
    target = [200, 50, 100, 0];

    fprintf('Target (Internal): X=%.2f, Y=%.2f, Z=%.2f, Pitch=%.2f\n', target);

    % IK
    q_sol = OpenManipulator.IK(target(1), target(2), target(3), target(4));
    fprintf('IK Sol (deg): [%.2f, %.2f, %.2f, %.2f]\n', q_sol);

    % Check FK
    [T_sol, global_transforms] = OpenManipulator.FK(q_sol);
    p_int = T_sol(1:3, 4);

    % Validation (Unified Coordinates)
    p_user = [p_int(1), p_int(2), p_int(3)];

    fprintf('FK(IK) Result (Internal): X=%.2f, Y=%.2f, Z=%.2f\n', p_user);

    err = norm(p_user - target(1:3));
    fprintf('Error: %.4f mm\n', err);

    if err < 1.0
        fprintf('PASS: Round-trip successful.\n');
    else
        error('FAIL: Error too large.');
    end

    % Check Elbow Down
    % T2 (Frame 2) height
    T2 = global_transforms(:,:,2);
    fprintf('Elbow Z: %.2f\n', T2(3, 4));

catch ME
    fprintf('ERROR: %s\n', ME.message);
    exit(1);
end

exit(0);
