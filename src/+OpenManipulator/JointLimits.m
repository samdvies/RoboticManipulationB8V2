classdef JointLimits
%JOINTLIMITS Joint limits for safe operation of OpenManipulator-X
%   Centralised joint limit definitions and clamping utilities.

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

        function [q_clamped, was_clamped] = Clamp(q, warn, min_delta_warn_deg)
        %CLAMP Clamp joint angles to safe limits
        %   [q_clamped, was_clamped] = Clamp(q)
        %   [q_clamped, was_clamped] = Clamp(q, warn, min_delta_warn_deg)
            if nargin < 2 || isempty(warn), warn = true; end
            if nargin < 3 || isempty(min_delta_warn_deg), min_delta_warn_deg = 0.05; end

            limits = OpenManipulator.JointLimits.GetLimits();
            names = OpenManipulator.JointLimits.GetNames();
            q = q(:)';
            q_clamped = q;
            was_clamped = false(1, 4);

            for i = 1:4
                if q(i) < limits(i, 1)
                    q_clamped(i) = limits(i, 1);
                    was_clamped(i) = true;
                elseif q(i) > limits(i, 2)
                    q_clamped(i) = limits(i, 2);
                    was_clamped(i) = true;
                end

                if was_clamped(i)
                    delta = abs(q(i) - q_clamped(i));
                    if warn && delta >= min_delta_warn_deg
                        fprintf('Warning: Joint %d (%s) clamped: %.3f deg -> %.3f deg (delta=%.3f, limit=[%.1f, %.1f] deg)\n', ...
                            i, names{i}, q(i), q_clamped(i), delta, limits(i, 1), limits(i, 2));
                    end
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
