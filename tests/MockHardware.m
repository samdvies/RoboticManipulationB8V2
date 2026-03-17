classdef MockHardware < handle
    properties
        calls = {}
    end

    methods
        function moveToPose(obj, x, y, z, pitch, time_sec, mode, z_floor_mm, varargin)
            call = struct( ...
                'type', 'moveToPose', ...
                'x', x, ...
                'y', y, ...
                'z', z, ...
                'pitch', pitch, ...
                'time_sec', time_sec, ...
                'mode', mode, ...
                'z_floor_mm', z_floor_mm);
            if ~isempty(varargin)
                call.extra = varargin;
            end
            obj.calls{end + 1} = call;
        end

        function openGripper(obj)
            obj.calls{end + 1} = struct('type', 'openGripper');
        end

        function closeGripper(obj)
            obj.calls{end + 1} = struct('type', 'closeGripper');
        end

        function setGripperPosition(obj, pct)
            obj.calls{end + 1} = struct('type', 'setGripperPosition', 'pct', pct);
        end
    end
end
