classdef JointLimits
%JOINTLIMITS Joint limits for safe operation of OpenManipulator-X
%   Centralised joint limit definitions and clamping utilities.
%
%   Joint 1 (Base):     ±90°  — User restriction: front half-plane only
%   Joint 2 (Shoulder): ±117° — Conservative safe default
%   Joint 3 (Elbow):    ±117° — Conservative safe default
%   Joint 4 (Wrist):    ±117° — Conservative safe default

    methods (Static)

        function limits = GetLimits()
        %GETLIMITS Returns 4x2 matrix [min, max] in degrees
            limits = [-90,  90;
                     -117, 117;
                     -117, 117;
                     -117, 117];
        end

        function names = GetNames()
        %GETNAMES Returns cell array of joint names
            names = {'Base', 'Shoulder', 'Elbow', 'Wrist'};
        end

        function [q_clamped, was_clamped] = Clamp(q)
        %CLAMP Clamp joint angles to safe limits
        %   [q_clamped, was_clamped] = Clamp(q)
        %
        %   Input:  q - 1x4 joint angles in degrees
        %   Output: q_clamped   - 1x4 clamped angles
        %           was_clamped - 1x4 logical array
            limits = OpenManipulator.JointLimits.GetLimits();
            names = OpenManipulator.JointLimits.GetNames();
            q = q(:)';
            q_clamped = q;
            was_clamped = false(1, 4);

            for i = 1:4
                if q(i) < limits(i, 1)
                    q_clamped(i) = limits(i, 1);
                    was_clamped(i) = true;
                    fprintf('Warning: Joint %d (%s) clamped: %.1f° -> %.1f° (limit: [%.0f°, %.0f°])\n', ...
                        i, names{i}, q(i), q_clamped(i), limits(i, 1), limits(i, 2));
                elseif q(i) > limits(i, 2)
                    q_clamped(i) = limits(i, 2);
                    was_clamped(i) = true;
                    fprintf('Warning: Joint %d (%s) clamped: %.1f° -> %.1f° (limit: [%.0f°, %.0f°])\n', ...
                        i, names{i}, q(i), q_clamped(i), limits(i, 1), limits(i, 2));
                end
            end
        end

        function [is_valid, violations] = Validate(q)
        %VALIDATE Check if joint angles are within limits
        %   [is_valid, violations] = Validate(q)
            limits = OpenManipulator.JointLimits.GetLimits();
            names = OpenManipulator.JointLimits.GetNames();
            q = q(:)';
            violations = {};

            for i = 1:4
                if q(i) < limits(i, 1) || q(i) > limits(i, 2)
                    violations{end+1} = struct('joint', i, 'name', names{i}, ...
                        'angle', q(i), 'min', limits(i, 1), 'max', limits(i, 2)); %#ok<AGROW>
                end
            end

            is_valid = isempty(violations);
        end

    end
end
