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
        ADDR_PROFILE_VELOCITY   = 112;
        ADDR_GOAL_POSITION      = 116;
        ADDR_MOVING             = 122;
        ADDR_PRESENT_POSITION   = 132;

        % Constants
        POSITION_CONTROL_MODE = 3;
        HOME_ENCODER = 2048;
        ENCODER_PER_DEG = 4096 / 360;
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

                % Set profile velocity
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_PROFILE_VELOCITY, velocity);

                % Check errors
                dxl_comm = getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
                if dxl_comm ~= 0
                    warning('Motor ID %d: comm error (code %d)', id, dxl_comm);
                else
                    fprintf('  Motor %d (%s): OK [%d°, %d°] vel=%d\n', ...
                        id, joint_names{i}, limits(i,1), limits(i,2), velocity);
                end
            end
            fprintf('--- Configuration Complete ---\n\n');
        end

        function enableTorque(obj)
        %ENABLETORQUE Enable torque on all 4 arm joints
            for id = obj.DXL_IDS
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_TORQUE_ENABLE, 1);
                pause(0.05);
            end
            fprintf('Torque ENABLED on all joints.\n');
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

        function moveToAnglesInterpolated(obj, q_target, num_steps)
        %MOVETOANGLESINTERPOLATED Linear interpolation from current to target
        %   moveToAnglesInterpolated([q1,q2,q3,q4], num_steps)
        %
        %   Generates num_steps linearly spaced waypoints and sync-writes each.
            if nargin < 3
                num_steps = 10;
            end

            q_current = obj.readAngles();

            for step = 1:num_steps
                t = step / num_steps;
                q_interp = (1 - t) * q_current + t * q_target;

                encoders = zeros(1, 4);
                for i = 1:4
                    encoders(i) = obj.deg2enc(q_interp(i));
                end

                obj.syncWritePositions(encoders);
                pause(0.05);  % 50ms between waypoints
            end

            % Wait for final position to settle
            obj.waitForMotion();
        end

        function moveHome(obj)
        %MOVEHOME Move all joints to home position (encoder 2048)
            fprintf('Moving to HOME position...\n');
            home_encoders = [obj.HOME_ENCODER, obj.HOME_ENCODER, ...
                             obj.HOME_ENCODER, obj.HOME_ENCODER];
            obj.syncWritePositions(home_encoders);
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
