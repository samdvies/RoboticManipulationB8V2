classdef HardwareInterface < handle
%HARDWAREINTERFACE DYNAMIXEL SDK wrapper for OpenManipulator-X
%   Provides sync write, linear interpolation, and safe motor control.
%
%   Usage:
%       hw = OpenManipulator.HardwareInterface('COM4', 1000000);
%       hw.configure(20);       % Velocity = 20 (~4.6 RPM)
%       hw.enableTorque();
%       hw.moveHome();
%       hw.moveToAnglesInterpolated([45, -30, 60, 10], 10);
%       hw.disconnect();

    properties (Constant)
        % Motor IDs
        DXL_IDS = [11, 12, 13, 14];
        GRIPPER_ID = 15;

        % Protocol
        PROTOCOL_VERSION = 2.0;

        % Control Table Addresses (XM430-W350-T)
        ADDR_OPERATING_MODE     = 11;
        ADDR_MIN_POSITION_LIMIT = 52;
        ADDR_MAX_POSITION_LIMIT = 48;
        ADDR_TORQUE_ENABLE      = 64;
        ADDR_PROFILE_ACCELERATION = 108;
        ADDR_PROFILE_VELOCITY   = 112;
        ADDR_GOAL_POSITION      = 116;
        ADDR_MOVING             = 122;
        ADDR_PRESENT_POSITION   = 132;

        % Constants
        POSITION_CONTROL_MODE = 3;
        HOME_ENCODER = 2048;
        ENCODER_PER_DEG = 4096 / 360;

        % Gripper encoder limits (from Dynamixel Wizard: open=120°, closed=240°)
        GRIPPER_OPEN_ENC  = 1365;  % 120° fully open
        GRIPPER_CLOSE_ENC = 2276;  % 200° closed (cube pickup)
    end

    properties
        port_num    % Port handler
        lib_name    % SDK library name
        group_num   % Sync write group handler
        is_connected = false;
    end

    methods
        function obj = HardwareInterface(com_port, baudrate)
        %HARDWAREINTERFACE Constructor — connects to robot
        %   hw = HardwareInterface('COM4', 1000000)
            if nargin < 2
                baudrate = 1000000;
            end
            obj.connect(com_port, baudrate);
        end

        function connect(obj, com_port, baudrate)
        %CONNECT Load SDK library and open serial port
            % Determine library name
            if strcmp(computer, 'PCWIN64')
                obj.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNXA64')
                obj.lib_name = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
                obj.lib_name = 'libdxl_mac_c';
            else
                error('Unsupported platform: %s', computer);
            end

            % Load library if not already loaded
            if ~libisloaded(obj.lib_name)
                fprintf('Loading Dynamixel SDK: %s\n', obj.lib_name);
                [~, ~] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', ...
                    'addheader', 'port_handler.h', ...
                    'addheader', 'packet_handler.h', ...
                    'addheader', 'group_sync_write.h', ...
                    'addheader', 'group_sync_read.h');
            end

            % Open port
            fprintf('Opening port: %s at %d baud\n', com_port, baudrate);
            obj.port_num = portHandler(com_port);
            packetHandler();

            if ~openPort(obj.port_num)
                error('Failed to open port %s.', com_port);
            end

            if ~setBaudRate(obj.port_num, baudrate)
                error('Failed to set baud rate to %d.', baudrate);
            end

            % Create Sync Write group for Goal Position (4 bytes)
            obj.group_num = groupSyncWrite(obj.port_num, ...
                obj.PROTOCOL_VERSION, obj.ADDR_GOAL_POSITION, 4);

            obj.is_connected = true;
            fprintf('Connected successfully.\n');
        end

        function configure(obj, velocity)
        %CONFIGURE Set operating mode, velocity, and position limits
        %   configure(velocity) — velocity in DYNAMIXEL units (20 ≈ 4.6 RPM)
            if nargin < 2
                velocity = 20;
            end

            fprintf('\n--- Configuring Motors (velocity=%d) ---\n', velocity);

            % Joint limits in encoder units
            limits = OpenManipulator.JointLimits.GetLimits();
            joint_names = OpenManipulator.JointLimits.GetNames();

            for i = 1:4
                id = obj.DXL_IDS(i);

                % Disable torque first (required to change EEPROM settings)
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_TORQUE_ENABLE, 0);
                pause(0.05);

                % Set Position Control Mode
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_OPERATING_MODE, obj.POSITION_CONTROL_MODE);
                pause(0.05);

                % Set position limits from JointLimits
                min_enc = obj.deg2enc(limits(i, 1));
                max_enc = obj.deg2enc(limits(i, 2));
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_MIN_POSITION_LIMIT, min_enc);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_MAX_POSITION_LIMIT, max_enc);

                % Set profile velocity & acceleration
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_PROFILE_VELOCITY, velocity);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_PROFILE_ACCELERATION, max(1, round(velocity/2))); % Accel = 50% of Vel

                % Check errors
                dxl_comm = getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
                if dxl_comm ~= 0
                    warning('Motor ID %d: comm error (code %d)', id, dxl_comm);
                else
                    fprintf('  Motor %d (%s): OK [%d°, %d°] vel=%d\n', ...
                        id, joint_names{i}, limits(i,1), limits(i,2), velocity);
                end
            end

            % Configure gripper motor
            obj.configureGripper(velocity);

            fprintf('--- Configuration Complete ---\n\n');
        end

        function enableTorque(obj)
        %ENABLETORQUE Enable torque on all arm joints + gripper
            all_ids = [obj.DXL_IDS, obj.GRIPPER_ID];
            for id = all_ids
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_TORQUE_ENABLE, 1);
                pause(0.05);
            end
            fprintf('Torque ENABLED on all motors (arm + gripper).\n');
        end

        function disableTorque(obj)
        %DISABLETORQUE Disable torque on all motors (arm + gripper)
            all_ids = [obj.DXL_IDS, obj.GRIPPER_ID];
            for id = all_ids
                try
                    write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                        obj.ADDR_TORQUE_ENABLE, 0);
                catch
                    % Ignore errors during shutdown
                end
            end
            fprintf('Torque DISABLED on all motors.\n');
        end

        function syncWritePositions(obj, encoder_targets)
        %SYNCWRITEPOSITIONS Send all 4 goal positions in a single packet
        %   syncWritePositions([enc1, enc2, enc3, enc4])

            % Clear previous params
            groupSyncWriteClearParam(obj.group_num);

            % Add each motor's goal position
            for i = 1:4
                id = obj.DXL_IDS(i);
                enc = uint32(encoder_targets(i));
                groupSyncWriteAddParam(obj.group_num, id, enc, 4);
            end

            % Transmit
            groupSyncWriteTxPacket(obj.group_num);
        end

        function moveToAngles(obj, q)
        %MOVETOANGLES Move to joint angles (degrees), sync write, wait
        %   moveToAngles([q1, q2, q3, q4])
            encoders = zeros(1, 4);
            for i = 1:4
                encoders(i) = obj.deg2enc(q(i));
            end

            obj.syncWritePositions(encoders);
            obj.waitForMotion();
        end

        function moveToPose(obj, x, y, z, pitch, time_sec)
        %MOVETOPOSE High-level move command with Auto-Collision Avoidance
        %   moveToPose(x, y, z, pitch)
        %   moveToPose(x, y, z, pitch, time_sec)
        
            if nargin < 6, time_sec = 2.0; end
            
            % 1. Get Current Pose
            q_current = obj.readAngles();
            [T_current, ~] = OpenManipulator.FK(q_current);
            start_pos = T_current(1:3, 4);
            
            % 2. Calculate Target Angles
            try
                q_target = OpenManipulator.IK(x, y, z, pitch);
            catch
                error('IK Failed: Target [%.1f, %.1f, %.1f] unreachable.', x, y, z);
            end
            
            % 3. Simulate Direct Path
            MIN_Z = 25; % Safety margin for Elbow/Wrist
            is_safe = true;
            steps = 10;
            
            for i = 1:steps
                t = i / steps;
                q_sim = (1-t)*q_current + t*q_target;
                [~, tf] = OpenManipulator.FK(q_sim);
                
                % Check Elbow(2), Wrist(3) - Strict Safety
                % EE(5) is allowed to go low (picking/placing)
                if tf(3,4,2) < MIN_Z || tf(3,4,3) < MIN_Z 
                    is_safe = false;
                    break;
                end
                
                % Optional: Flag unsafe if EE dips below both Start Z and Target Z 
                % (U-shaped dip collision risk), but simple Elbow check is usually determining factor.
            end
            
            % 4. Execute
            if is_safe
                obj.moveToAnglesInterpolated(q_target, ceil(time_sec/0.05));
            else
                fprintf('  [AUTO-SAFETY] Direct path unsafe. Rerouting via Safe Z...\n');
                
                % Adaptive Safe Z Strategy
                % Base clearance: Above highest point of current or target
                safe_z = max(start_pos(3), z) + 40; 
                
                % If moving a significant distance, ensure absolute minimum height (to clear obstacles)
                dist = norm(start_pos - [x, y, z]);
                if dist > 50
                    safe_z = max(safe_z, 100); % Force at least 100mm for long moves
                end
                
                % Waypoint 1: Lift current X,Y to Safe Z
                q_via1 = OpenManipulator.IK(start_pos(1), start_pos(2), safe_z, pitch);
                obj.moveToAnglesInterpolated(q_via1, 20); 
                
                % Waypoint 2: Move to Target X,Y at Safe Z
                q_via2 = OpenManipulator.IK(x, y, safe_z, pitch);
                obj.moveToAnglesInterpolated(q_via2, 40); 
                
                % Waypoint 3: Lower to Target
                obj.moveToAnglesInterpolated(q_target, 20); 
            end
        end

        function moveToAnglesInterpolated(obj, q_target, num_steps)
        %MOVETOANGLESINTERPOLATED Smart software interpolation
        %   Uses linear interpolation for large moves, but skips for small moves.
        %   Verifies final position using FK.
        
            q_current = obj.readAngles();
            
            % Calculate max joint movement
            max_diff = max(abs(q_target - q_current));
            
            % Determine steps dynamicallly if not provided or for small moves
            if max_diff < 10
                 % Small move: Do it in 1 step to avoid jerkiness
                 num_steps = 1;
            elseif nargin < 3
                 % Default: ~5 degrees per step
                 num_steps = ceil(max_diff / 5);
            end
            
            % Ensure at least 1 step
            num_steps = max(1, num_steps);

            % Interpolation Loop
            for step = 1:num_steps
                t = step / num_steps;
                q_interp = (1 - t) * q_current + t * q_target;
                
                % --- Safety Check ---
                % Perform FK to check for ground collision
                [~, global_transforms] = OpenManipulator.FK(q_interp);
                
                % Check Elbow (Frame 2), Wrist (Frame 3), and End-Effector (Frame 5)
                % Z-heights are in the 3rd row, 4th column (Position Z)
                z_elbow = global_transforms(3, 4, 2);
                z_wrist = global_transforms(3, 4, 3);
                z_ee    = global_transforms(3, 4, 5);
                
                % Define Safety Threshold for Structure (Elbow/Wrist)
                MIN_Z_HEIGHT = 20; 
                
                % Relaxed Threshold for End Effector (allow to touch table/objects)
                MIN_EE_HEIGHT = 5; 
                
                if z_elbow < MIN_Z_HEIGHT || z_wrist < MIN_Z_HEIGHT || z_ee < MIN_EE_HEIGHT
                    error('Motion Safety Violation: Structure < %dmm or EE < %dmm. Aborting.', MIN_Z_HEIGHT, MIN_EE_HEIGHT);
                end
                % --------------------

                encoders = zeros(1, 4);
                for i = 1:4
                    encoders(i) = obj.deg2enc(q_interp(i));
                end

                obj.syncWritePositions(encoders);
                pause(0.05);  % 50ms between waypoints
            end

            % Wait for final position to settle
            obj.waitForMotion();
            
            % Verification
            obj.verifyPose(q_target);
        end

        function verifyPose(obj, q_target)
        %VERIFYPOSE Checks if actual robot pose matches target
             q_actual = obj.readAngles();
             try
                 [T_target, ~] = OpenManipulator.FK(q_target);
                 [T_actual, ~] = OpenManipulator.FK(q_actual);
                 
                 pos_target = T_target(1:3, 4);
                 pos_actual = T_actual(1:3, 4);
                 
                 dist = norm(pos_target - pos_actual);
                 
                 if dist > 15 % 15mm tolerance
                     fprintf('  [WARNING] Position Mismatch: %.1f mm error.\n', dist);
                     fprintf('    Target: [%.1f, %.1f, %.1f]\n', pos_target');
                     fprintf('    Actual: [%.1f, %.1f, %.1f]\n', pos_actual');
                 else
                     fprintf('  [Position Verified] Error: %.1f mm\n', dist);
                 end
             catch ME
                 % Ignore FK errors (e.g. if FK not in path)
             end
        end

        function moveHome(obj)
        %MOVEHOME Move all joints to home position (encoder 2048) and open gripper
            fprintf('Moving to HOME position...\n');
            home_encoders = [obj.HOME_ENCODER, obj.HOME_ENCODER, ...
                             obj.HOME_ENCODER, obj.HOME_ENCODER];
            obj.syncWritePositions(home_encoders);
            obj.openGripper();
            obj.waitForMotion();
            fprintf('HOME reached.\n');
        end

        function q = readAngles(obj)
        %READANGLES Read present joint angles in degrees
        %   q = readAngles() returns [q1, q2, q3, q4] in degrees
            q = zeros(1, 4);
            for i = 1:4
                enc = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                    obj.DXL_IDS(i), obj.ADDR_PRESENT_POSITION);
                q(i) = obj.enc2deg(enc);
            end
        end

        function waitForMotion(obj, timeout)
        %WAITFORMOTION Poll ADDR_MOVING until all motors stopped
            if nargin < 2
                timeout = 10.0;
            end

            start_time = tic;
            while toc(start_time) < timeout
                all_stopped = true;
                for id = obj.DXL_IDS
                    moving = read1ByteTxRx(obj.port_num, ...
                        obj.PROTOCOL_VERSION, id, obj.ADDR_MOVING);
                    if moving == 1
                        all_stopped = false;
                        break;
                    end
                end

                if all_stopped
                    % Double-check stability
                    pause(0.1);
                    still_stopped = true;
                    for id = obj.DXL_IDS
                        moving = read1ByteTxRx(obj.port_num, ...
                            obj.PROTOCOL_VERSION, id, obj.ADDR_MOVING);
                        if moving == 1
                            still_stopped = false;
                            break;
                        end
                    end
                    if still_stopped
                        return;
                    end
                end
                pause(0.05);
            end
            warning('Motion timeout after %.1f seconds.', timeout);
        end

        function configureGripper(obj, velocity)
        %CONFIGUREGRIPPER Configure gripper motor (ID 15)
        %   configureGripper(velocity)
            if nargin < 2
                velocity = 20;
            end

            id = obj.GRIPPER_ID;

            fprintf('  Configuring Gripper (ID %d)...\n', id);

            % 1. Disable torque (Required to change Operating Mode)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                obj.ADDR_TORQUE_ENABLE, 0);
            pause(0.1);

            % 2. Set Operating Mode to Position Control (Mode 3)
            % The diagnostic showing Mode 1 means it was likely in Velocity mode
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                obj.ADDR_OPERATING_MODE, obj.POSITION_CONTROL_MODE);
            pause(0.1);

            % Verify Mode
            curr_mode = read1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                obj.ADDR_OPERATING_MODE);
            if curr_mode ~= obj.POSITION_CONTROL_MODE
                warning('Gripper Operating Mode failed to set! Readings: %d', curr_mode);
            end

            % 3. Set Position Limits
            min_enc = min(obj.GRIPPER_CLOSE_ENC, obj.GRIPPER_OPEN_ENC);
            max_enc = max(obj.GRIPPER_CLOSE_ENC, obj.GRIPPER_OPEN_ENC);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                obj.ADDR_MIN_POSITION_LIMIT, min_enc);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                obj.ADDR_MAX_POSITION_LIMIT, max_enc);

            % 4. Set Profile Velocity
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                obj.ADDR_PROFILE_VELOCITY, velocity);

            % 5. Enable Torque (Gripper must be explicitly enabled here if not done globally)
            % Note: enableTorque() method does this for all, but good to be safe here?
            % Actually, standard practice is to configure first, then enable.
            
            dxl_comm = getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
            if dxl_comm ~= 0
                warning('Gripper motor ID %d: comm error (code %d)', id, dxl_comm);
            else
                fprintf('  Motor %d (Gripper): OK [Mode %d] [enc %d–%d] vel=%d\n', ...
                    id, curr_mode, min_enc, max_enc, velocity);
            end
        end

        function setGripperPosition(obj, pct)
        %SETGRIPPERPOSITION Set gripper to a percentage position
        %   setGripperPosition(pct) — 0 = fully open, 100 = fully closed
            pct = max(0, min(100, pct));
            enc = round(obj.GRIPPER_OPEN_ENC + ...
                (pct / 100) * (obj.GRIPPER_CLOSE_ENC - obj.GRIPPER_OPEN_ENC));
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.GRIPPER_ID, obj.ADDR_GOAL_POSITION, enc);
            fprintf('Gripper -> %d%% (encoder %d)\n', pct, enc);
        end

        function pct = readGripperPosition(obj)
        %READGRIPPERPOSITION Read current gripper position as 0–100%
        %   pct = readGripperPosition() — 0 = open, 100 = closed
            enc = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.GRIPPER_ID, obj.ADDR_PRESENT_POSITION);
            range = obj.GRIPPER_CLOSE_ENC - obj.GRIPPER_OPEN_ENC;
            pct = (double(enc) - obj.GRIPPER_OPEN_ENC) / range * 100;
            pct = max(0, min(100, pct));
        end

        function openGripper(obj)
        %OPENGRIPPER Fully open the gripper
            obj.setGripperPosition(0);
        end

        function closeGripper(obj)
        %CLOSEGRIPPER Fully close the gripper
            obj.setGripperPosition(100);
        end

        function disconnect(obj)
        %DISCONNECT Disable torque, close port
            if obj.is_connected
                fprintf('\n>>> Shutting down...\n');
                obj.disableTorque();
                try
                    closePort(obj.port_num);
                    fprintf('>>> Port closed.\n');
                catch
                end
                obj.is_connected = false;
                fprintf('>>> Disconnected.\n');
            end
        end

        function delete(obj)
        %DELETE Destructor — ensures clean shutdown
            obj.disconnect();
        end
    end

    methods (Static)
        function enc = deg2enc(deg)
        %DEG2ENC Convert degrees (from home) to DYNAMIXEL encoder value
        %   encoder = (deg / 360) * 4096 + 2048
            enc = round((deg / 360) * 4096) + 2048;
            enc = max(0, min(4095, enc));
        end

        function deg = enc2deg(enc)
        %ENC2DEG Convert DYNAMIXEL encoder value to degrees (from home)
        %   deg = ((encoder - 2048) / 4096) * 360
            deg = ((double(enc) - 2048) / 4096) * 360;
        end
    end
end
