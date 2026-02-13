function progressiveMotorTest(port_num, lib_name)
% PROGRESSIVEMOTORTEST Safely tests each motor incrementally with user confirmation
%
% This function enables and tests one motor at a time, allowing the user to
% verify safe operation before proceeding to the next motor.
%
% TEST SEQUENCE:
%   1. Enable motor with torque
%   2. Move to +30° from home
%   3. Move to -30° from home
%   4. Return to home (0°)
%   5. Wait for user confirmation ('y' to continue, 'q' to stop)
%   6. Disable motor and proceed to next
%
% CONTROLS:
%   'y' or 'Y' - Continue to next motor
%   'q' or 'Q' - Emergency stop all motors and exit
%   Any other key - Repeat current motor test
%
% Usage:
%   [port_num, lib_name, cleanup] = robotSafeInit('COM3');
%   progressiveMotorTest(port_num, lib_name);
%
% Inputs:
%   port_num - Port handler from robotSafeInit
%   lib_name - Library name from robotSafeInit
%
% Author: OpenManipulator-X FK Simulation
% Date: February 2026

    %% Configuration
    PROTOCOL_VERSION = 2.0;
    DXL_IDS = [11, 12, 13, 14];  % Test arm joints only (not gripper)
    JOINT_NAMES = {'Base (ID 11)', 'Shoulder (ID 12)', 'Elbow (ID 13)', 'Wrist (ID 14)'};
    
    % Control Table Addresses
    ADDR_TORQUE_ENABLE = 64;
    ADDR_GOAL_POSITION = 116;
    ADDR_PRESENT_POSITION = 132;
    
    % Test positions (encoder values)
    HOME_POS = 2048;           % 0° (180° in Wizard)
    TEST_POS_PLUS = 2389;      % +30° from home (≈210° in Wizard)
    TEST_POS_MINUS = 1707;     % -30° from home (≈150° in Wizard)
    
    % Movement timing
    MOVE_DELAY = 1.5;          % Seconds to wait for movement completion
    
    %% Setup Figure for Keyboard Input
    fig = figure('Name', 'Progressive Motor Test', 'NumberTitle', 'off', ...
        'Position', [100, 100, 600, 400], 'Color', 'w');
    
    %% Test Each Motor
    fprintf('\n========================================\n');
    fprintf('   PROGRESSIVE MOTOR TEST\n');
    fprintf('========================================\n');
    fprintf('Controls:\n');
    fprintf('  ''y'' - Motor OK, continue to next\n');
    fprintf('  ''q'' - Emergency stop and exit\n');
    fprintf('  Any other key - Repeat test\n');
    fprintf('========================================\n\n');
    
    for i = 1:length(DXL_IDS)
        id = DXL_IDS(i);
        joint_name = JOINT_NAMES{i};
        
        test_complete = false;
        
        while ~test_complete
            %% Display Test Info
            clf(fig);
            annotation('textbox', [0.1, 0.5, 0.8, 0.4], ...
                'String', {sprintf('Testing: %s', joint_name), '', ...
                          'Movement sequence:', ...
                          '  1. Home → +30°', ...
                          '  2. +30° → -30°', ...
                          '  3. -30° → Home', '', ...
                          'Watch the robot carefully!'}, ...
                'FontSize', 14, 'HorizontalAlignment', 'center', ...
                'EdgeColor', 'none', 'FitBoxToText', 'off');
            drawnow;
            
            fprintf('\n--- Testing %s ---\n', joint_name);
            
            %% Enable Torque
            fprintf('Enabling torque...\n');
            write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE, 1);
            pause(0.1);
            
            %% Movement Test Sequence
            try
                % Move to home first (in case it's not there)
                fprintf('Moving to HOME (0°)...\n');
                write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, HOME_POS);
                pause(MOVE_DELAY);
                
                % Move to +30°
                fprintf('Moving to +30°...\n');
                write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, TEST_POS_PLUS);
                pause(MOVE_DELAY);
                
                % Move to -30°
                fprintf('Moving to -30°...\n');
                write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, TEST_POS_MINUS);
                pause(MOVE_DELAY);
                
                % Return to home
                fprintf('Returning to HOME...\n');
                write4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_GOAL_POSITION, HOME_POS);
                pause(MOVE_DELAY);
                
                % Read final position
                present_pos = read4ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRESENT_POSITION);
                present_deg = (present_pos - 2048) / 4096 * 360;
                fprintf('Final position: %d (%.1f° from home)\n', present_pos, present_deg);
                
            catch ME
                fprintf('ERROR during movement: %s\n', ME.message);
            end
            
            %% Keep Torque Enabled at Home (prevents arm falling)
            fprintf('Holding at HOME position (torque ON).\n');
            
            %% Wait for User Input
            clf(fig);
            annotation('textbox', [0.1, 0.3, 0.8, 0.5], ...
                'String', {sprintf('%s Test Complete', joint_name), '', ...
                          'Press a key:', ...
                          '  ''y'' - Motor OK, continue', ...
                          '  ''q'' - Emergency stop', ...
                          '  Other - Repeat test'}, ...
                'FontSize', 16, 'HorizontalAlignment', 'center', ...
                'EdgeColor', 'none', 'FitBoxToText', 'off', ...
                'BackgroundColor', [0.9, 1, 0.9]);
            drawnow;
            
            fprintf('\nPress ''y'' to continue, ''q'' to quit, or any key to repeat: ');
            
            % Wait for keypress
            waitforbuttonpress;
            key = get(fig, 'CurrentCharacter');
            
            if strcmpi(key, 'y')
                fprintf('Continuing to next motor...\n');
                test_complete = true;
                
            elseif strcmpi(key, 'q')
                fprintf('\n!!! EMERGENCY STOP REQUESTED !!!\n');
                % Disable all motors
                for stop_id = DXL_IDS
                    write1ByteTxRx(port_num, PROTOCOL_VERSION, stop_id, ADDR_TORQUE_ENABLE, 0);
                end
                close(fig);
                fprintf('All motors disabled. Test aborted.\n');
                return;
                
            else
                fprintf('Repeating test for %s...\n', joint_name);
                % Loop will repeat
            end
        end
    end
    
    %% All Tests Complete
    clf(fig);
    annotation('textbox', [0.1, 0.3, 0.8, 0.5], ...
        'String', {'ALL MOTOR TESTS COMPLETE!', '', ...
                  'All joints verified working correctly.', ...
                  'Robot is ready for use.'}, ...
        'FontSize', 16, 'HorizontalAlignment', 'center', ...
        'EdgeColor', 'none', 'FitBoxToText', 'off', ...
        'BackgroundColor', [0.8, 1, 0.8]);
    drawnow;
    
    fprintf('\n========================================\n');
    fprintf('   ALL MOTOR TESTS COMPLETE!\n');
    fprintf('========================================\n');
    fprintf('Robot is ready for operation.\n\n');
    
    pause(2);
    close(fig);
end
