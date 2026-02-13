function waypoints = smoothTrajectory(start_pos, end_pos, via_height, num_points)
% SMOOTHTRAJECTORY Generate smooth waypoints between two positions
%
% Creates a trajectory that lifts up, moves horizontally, then descends.
% Optionally generates interpolated waypoints for smooth motion.
%
% Inputs:
%   start_pos  - Starting position [x, y, z] in mm
%   end_pos    - End position [x, y, z] in mm  
%   via_height - Height to lift to during transfer (mm above start/end z)
%   num_points - Number of interpolation points (default: 5)
%
% Outputs:
%   waypoints  - Nx3 matrix of waypoint positions
%
% Example:
%   start = [200, 100, 25];
%   goal = [150, -50, 0];
%   wp = smoothTrajectory(start, goal, 100, 10);
%
% Motion Profile:
%   1. Lift straight up from start
%   2. Move horizontally at via height
%   3. Descend to end position
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

if nargin < 4
    num_points = 5;  % Default interpolation points per segment
end

%% Define Key Waypoints
% Lift point (above start)
max_z = max(start_pos(3), end_pos(3)) + via_height;
lift_pos = [start_pos(1), start_pos(2), max_z];

% Transit point (above end)
approach_pos = [end_pos(1), end_pos(2), max_z];

%% Build Waypoint Sequence
% Segment 1: Lift
seg1 = interpolateLinear(start_pos, lift_pos, num_points);

% Segment 2: Horizontal transit
seg2 = interpolateLinear(lift_pos, approach_pos, num_points);

% Segment 3: Descend
seg3 = interpolateLinear(approach_pos, end_pos, num_points);

% Combine (remove duplicates at segment boundaries)
waypoints = [seg1; seg2(2:end, :); seg3(2:end, :)];

end

%% Linear Interpolation Helper
function points = interpolateLinear(p1, p2, n)
    % Generate n linearly interpolated points from p1 to p2
    t = linspace(0, 1, n)';
    points = (1-t) .* p1 + t .* p2;
end
