% TEST_BRIDGE_ROTATION_STAGING_SEQUENCE
%
% Verifies the bridge-to-rotation staging handoff sequence:
% 1. Place staged cube while still at pitch 0
% 2. Open gripper before any pitch change
% 3. Retract upward at pitch 0
% 4. Only then change to transit pitch for the standard re-pick path

clc; clear;

test_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(test_dir, '..');
addpath(genpath(fullfile(project_root, 'src')));
addpath(genpath(fullfile(project_root, 'Common')));
addpath(fullfile(project_root, 'scripts'));
addpath(test_dir);

cfg = task_config();
hw = MockHardware();

target_xy = [150, -150];
pickup_angle_deg = 0;

placeBridgeRotationStaging(hw, target_xy, pickup_angle_deg, cfg);

calls = hw.calls;
assert(~isempty(calls), 'Expected recorded hardware calls.');

call_types = cellfun(@(c) c.type, calls, 'UniformOutput', false);
move_calls = calls(strcmp(call_types, 'moveToPose'));
assert(numel(move_calls) >= 4, 'Expected at least four moveToPose calls.');

open_idx = find(strcmp(call_types, 'openGripper'), 1);
assert(~isempty(open_idx), 'Expected gripper release during staging.');

first_nonzero_pitch_after_release = [];
for i = open_idx + 1:numel(calls)
    if strcmp(calls{i}.type, 'moveToPose') && abs(calls{i}.pitch) > 1e-6
        first_nonzero_pitch_after_release = i;
        break;
    end
end
assert(~isempty(first_nonzero_pitch_after_release), 'Expected a post-release pitch change.');

retract_idx = open_idx + 1;
assert(strcmp(calls{retract_idx}.type, 'moveToPose'), 'Expected retract move immediately after release.');
assert(abs(calls{retract_idx}.pitch) < 1e-6, 'Retract after release must keep pitch at 0.');
assert(calls{retract_idx}.z >= cfg.PLACE_APPROACH_Z - 1e-6, 'Retract after release must go to safe height.');

assert(first_nonzero_pitch_after_release > retract_idx, 'Pitch change must occur after vertical retract.');

fprintf('PASS: Bridge rotation staging releases before pitch change and retracts safely.\n');
