function [assignment, total_distance] = optimalMatching(cube_positions, holder_positions)
% OPTIMALMATCHING Find optimal assignment of cubes to holders
%
% Uses the Hungarian algorithm to minimize total travel distance
% when moving cubes to empty holders.
%
% Inputs:
%   cube_positions   - Nx3 matrix of cube positions [x, y, z]
%   holder_positions - Mx3 matrix of holder positions [x, y, z]
%
% Outputs:
%   assignment     - Vector where assignment(i) = holder index for cube i
%   total_distance - Total Euclidean distance for all assignments
%
% Example:
%   cubes = [200, 100, 25; 150, -50, 25; 100, 80, 25];
%   holders = [250, -100, 0; 180, 150, 0; 120, -80, 0];
%   [assign, dist] = optimalMatching(cubes, holders);
%
% Author: OpenManipulator-X Task 2
% Date: February 2026

num_cubes = size(cube_positions, 1);
num_holders = size(holder_positions, 1);

if num_cubes > num_holders
    warning('More cubes than holders! Some cubes cannot be assigned.');
end

%% Build Cost Matrix (Distance Matrix)
% cost_matrix(i,j) = distance from cube i to holder j
cost_matrix = zeros(num_cubes, num_holders);

for i = 1:num_cubes
    for j = 1:num_holders
        cost_matrix(i, j) = norm(cube_positions(i, :) - holder_positions(j, :));
    end
end

fprintf('Cost matrix (distances in mm):\n');
disp(round(cost_matrix));

%% Hungarian Algorithm
% Using MATLAB's matchpairs for optimal assignment (available in R2019b+)
% If not available, fall back to greedy assignment

try
    % matchpairs minimizes total cost
    M = matchpairs(cost_matrix, 1e6);  % Large unassigned cost
    assignment = zeros(num_cubes, 1);
    for k = 1:size(M, 1)
        assignment(M(k, 1)) = M(k, 2);
    end
catch
    % Fallback: Greedy assignment
    fprintf('Using greedy assignment (matchpairs not available)\n');
    assignment = greedyAssignment(cost_matrix);
end

%% Calculate Total Distance
total_distance = 0;
for i = 1:num_cubes
    if assignment(i) > 0
        total_distance = total_distance + cost_matrix(i, assignment(i));
    end
end

%% Display Results
fprintf('\nOptimal Assignment:\n');
for i = 1:num_cubes
    if assignment(i) > 0
        fprintf('  Cube %d -> Holder %d (distance: %.1f mm)\n', ...
            i, assignment(i), cost_matrix(i, assignment(i)));
    else
        fprintf('  Cube %d -> UNASSIGNED\n', i);
    end
end
fprintf('Total distance: %.1f mm\n', total_distance);

end

%% Greedy Assignment Fallback
function assignment = greedyAssignment(cost_matrix)
    [num_cubes, num_holders] = size(cost_matrix);
    assignment = zeros(num_cubes, 1);
    used_holders = false(num_holders, 1);
    
    % Sort all (cube, holder) pairs by distance
    pairs = [];
    for i = 1:num_cubes
        for j = 1:num_holders
            pairs = [pairs; i, j, cost_matrix(i, j)];
        end
    end
    [~, idx] = sort(pairs(:, 3));
    pairs = pairs(idx, :);
    
    % Assign greedily
    assigned_cubes = false(num_cubes, 1);
    for k = 1:size(pairs, 1)
        i = pairs(k, 1);
        j = pairs(k, 2);
        if ~assigned_cubes(i) && ~used_holders(j)
            assignment(i) = j;
            assigned_cubes(i) = true;
            used_holders(j) = true;
        end
        if all(assigned_cubes)
            break;
        end
    end
end
