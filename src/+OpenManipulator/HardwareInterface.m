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
        % COM debug: set true to log every Tx/Rx result and help diagnose port drops
        COM_DEBUG = false;
        COM_OP_COUNT = 0;   % Incremented on each check; helps correlate logs with time
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

            obj.logCom('[CONNECT] portHandler done, opening port...');
            if ~openPort(obj.port_num)
                error('Failed to open port %s. (Device in use? Wrong COM number? Unplugged?)', com_port);
            end
            obj.logCom('[CONNECT] openPort OK');

            if ~setBaudRate(obj.port_num, baudrate)
                closePort(obj.port_num);
                error('Failed to set baud rate to %d on %s.', baudrate, com_port);
            end
            obj.logCom('[CONNECT] setBaudRate OK');

            % Create Sync Write group for Goal Position (4 bytes)
            obj.group_num = groupSyncWrite(obj.port_num, ...
                obj.PROTOCOL_VERSION, obj.ADDR_GOAL_POSITION, 4);

            obj.is_connected = true;
            obj.com_tic = tic;
            fprintf('Connected successfully (port %s @ %d baud).\n', com_port, baudrate);
        end

        function logCom(obj, msg)
        %LOGCOM If COM_DEBUG, print timestamped COM message; always increment op count
            obj.COM_OP_COUNT = obj.COM_OP_COUNT + 1;
            if obj.COM_DEBUG
                t = 0;
                if ~isempty(obj.com_tic), t = toc(obj.com_tic); end
                fprintf('[COM #%d t=%.2f] %s\n', obj.COM_OP_COUNT, t, msg);
            end
        end

        function ok = checkAndLogComResult(obj, caller_name, motor_id)
        %CHECKANDLOGCOMRESULT Check getLastTxRxResult/getLastRxPacketError; log if COM_DEBUG or non-zero
            if nargin < 3
                motor_id = [];
            end
            comm = getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
            err = getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION);
            obj.COM_OP_COUNT = obj.COM_OP_COUNT + 1;
            ok = (comm == 0 && err == 0);
            id_str = '';
            if ~isempty(motor_id)
                id_str = sprintf(' id=%d', motor_id);
            end
            if ~ok || obj.COM_DEBUG
                comm_str = obj.commResultString(comm);
                err_str = obj.rxErrorString(err);
                t = 0; if ~isempty(obj.com_tic), t = toc(obj.com_tic); end
                fprintf('[COM #%d t=%.2f] %s%s TxRx=%d (%s) RxErr=0x%02X (%s)\n', ...
                    obj.COM_OP_COUNT, t, caller_name, id_str, comm, comm_str, err, err_str);
            end
        end

        function s = commResultString(~, code)
            switch code
                case 0,   s = 'SUCCESS';
                case 1,   s = 'PORT_BUSY';
                case 2,   s = 'TX_FAIL';
                case 3,   s = 'RX_FAIL';
                case 4,   s = 'TX_ERROR';
                case 5,   s = 'RX_WAITING';
                case 256, s = 'RX_TIMEOUT';
                case 512, s = 'TX_TIMEOUT';
                otherwise, s = sprintf('UNKNOWN_%d', code);
            end
        end

        function s = rxErrorString(~, bits)
            if bits == 0
                s = 'none';
                return;
            end
            parts = {};
            if bitand(bits, 1),  parts{end+1} = 'INPUT_VOLTAGE'; end
            if bitand(bits, 2),  parts{end+1} = 'ANGLE_LIMIT'; end
            if bitand(bits, 4),  parts{end+1} = 'OVERHEAT'; end
            if bitand(bits, 8),  parts{end+1} = 'RANGE'; end
            if bitand(bits, 16), parts{end+1} = 'CHECKSUM'; end
            if bitand(bits, 32), parts{end+1} = 'OVERLOAD'; end
            if bitand(bits, 64), parts{end+1} = 'INSTRUCTION'; end
            if isempty(parts), s = sprintf('0x%02X', bits); else, s = strjoin(parts, '|'); end
        end

        function startComTic(obj)
        %STARTCOMTIC Start/restart the timer used for COM_DEBUG timestamps (call after connect)
            obj.com_tic = tic;
        end

        function [ok, comm, rxerr] = checkComHealth(obj, verbose)
        %CHECKCOMHEALTH Quick port sanity check: read one motor (ID 11). Returns ok, comm code, rx error.
        %   [ok, comm, rxerr] = checkComHealth() or checkComHealth(true) for forced print
            if nargin < 2, verbose = false; end
            read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_IDS(1), obj.ADDR_PRESENT_POSITION);
            comm = getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
            rxerr = getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION);
            ok = (comm == 0 && rxerr == 0);
            if verbose || ~ok || obj.COM_DEBUG
                fprintf('[COM HEALTH #%d t=%.2f] read ID11: ok=%d comm=%d (%s) rxerr=0x%02X (%s)\n', ...
                    obj.COM_OP_COUNT + 1, toc(obj.com_tic), ok, comm, obj.commResultString(comm), rxerr, obj.rxErrorString(rxerr));
            end
            obj.COM_OP_COUNT = obj.COM_OP_COUNT + 1;
        end
    end

    properties (Access = private)
        com_tic = [];  % tic for COM_DEBUG elapsed time
    end

    methods

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
                obj.checkAndLogComResult('configure(torque_off)', id);
                pause(0.05);

                % Set Position Control Mode
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_OPERATING_MODE, obj.POSITION_CONTROL_MODE);
                obj.checkAndLogComResult('configure(mode)', id);
                pause(0.05);

                % Set position limits from JointLimits
                min_enc = obj.deg2enc(limits(i, 1));
                max_enc = obj.deg2enc(limits(i, 2));
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_MIN_POSITION_LIMIT, min_enc);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_MAX_POSITION_LIMIT, max_enc);
                obj.checkAndLogComResult('configure(limits)', id);

                % Set profile velocity & acceleration
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_PROFILE_VELOCITY, velocity);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, ...
                    obj.ADDR_PROFILE_ACCELERATION, max(1, round(velocity/2))); % Accel = 50% of Vel
                obj.checkAndLogComResult('configure(vel/accel)', id);

                % Summary
                dxl_comm = getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
                if dxl_comm ~= 0
                    warning('Motor ID %d: comm error (code %d) %s', id, dxl_comm, obj.commResultString(dxl_comm));
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

            % Transmit (Sync Write does not return status packet; TxRx result may be stale)
            groupSyncWriteTxPacket(obj.group_num);
            if obj.COM_DEBUG
                obj.logCom(sprintf('syncWritePositions enc=[%u %u %u %u]', encoder_targets(1), encoder_targets(2), encoder_targets(3), encoder_targets(4)));
            end
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

        function moveToPose(obj, x, y, z, pitch, time_sec, mode, z_floor_mm, ctx_or_fly)
        %MOVETOPOSE High-level move command with selectable motion mode
        %   moveToPose(x, y, z, pitch)
        %   moveToPose(x, y, z, pitch, time_sec)
        %   moveToPose(x, y, z, pitch, time_sec, mode, z_floor_mm)
        %   moveToPose(..., z_floor_mm, ctx_or_fly)
        %
        %   mode: 1=Joint, 2=Task-space linear, 3=Jacobian
        %   ctx_or_fly: struct with .preplanned_route, .zones, etc. = bridge_ctx (skip reroute, optional dynamic pitch);
        %               logical = is_fly (higher Cartesian speed for fly sections)
            if nargin < 6 || isempty(time_sec), time_sec = 2.0; end
            if nargin < 7 || isempty(mode), mode = 1; end
            if nargin < 8 || isempty(z_floor_mm), z_floor_mm = 20.0; end
            if nargin < 9 || isempty(ctx_or_fly)
                ctx_or_fly = false;
            end
            preplanned_route = false;
            if isstruct(ctx_or_fly) && isfield(ctx_or_fly, 'preplanned_route')
                preplanned_route = logical(ctx_or_fly.preplanned_route);
            end
            
            % 1. Get Current Pose
            q_current = obj.readAngles();
            [T_current, ~] = OpenManipulator.FK(q_current);
            start_pos = T_current(1:3, 4);
            % Keep pitch convention identical to IK/Python: pitch = -(q2 + q3 + q4)
            start_pitch = -(q_current(2) + q_current(3) + q_current(4));
            
            % 2. Calculate Target Angles
            try
                q_target = OpenManipulator.IK(x, y, z, pitch);
            catch
                error('IK Failed: Target [%.1f, %.1f, %.1f] unreachable.', x, y, z);
            end
            
            % 3. Simulate Direct Path (method depends on mode)
            MIN_Z = z_floor_mm; % End-effector floor limit only
            is_safe = true;
            steps = 10;

            if mode == 2 || mode == 3
                % Task-space linear / Jacobian: actual path follows Cartesian direction.
                % Check commanded pose Z along the segment; do not use joint-space sim (would wrongly trigger reroute).
                for i = 1:steps
                    s = i / steps;
                    pose_sim = (1 - s) * start_pos(:) + s * [x; y; z];
                    if pose_sim(3) < MIN_Z
                        is_safe = false;
                        break;
                    end
                end
            else
                % Joint only: simulate joint-space path for safety.
                for i = 1:steps
                    t = i / steps;
                    q_sim = (1-t)*q_current + t*q_target;
                    [T_sim, ~] = OpenManipulator.FK(q_sim);
                    if T_sim(3,4) < MIN_Z
                        is_safe = false;
                        break;
                    end
                end
            end
            
            % 4. Execute
            if preplanned_route
                obj.executePoseMove(q_target, [x, y, z], pitch, time_sec, mode, z_floor_mm, ctx_or_fly);
                return;
            end
            if is_safe
                obj.executePoseMove(q_target, [x, y, z], pitch, time_sec, mode, z_floor_mm, ctx_or_fly);
            else
                fprintf('  [AUTO-SAFETY] Direct path unsafe. Rerouting via Safe Z...\n');
                safe_z = max(start_pos(3), z) + 40;
                dist = norm(start_pos - [x, y, z]);
                if dist > 50
                    safe_z = max(safe_z, 100);
                end
                q_via1 = OpenManipulator.IK(start_pos(1), start_pos(2), safe_z, pitch);
                obj.executePoseMove(q_via1, [start_pos(1), start_pos(2), safe_z], pitch, time_sec, mode, z_floor_mm, ctx_or_fly);
                q_via2 = OpenManipulator.IK(x, y, safe_z, pitch);
                obj.executePoseMove(q_via2, [x, y, safe_z], pitch, time_sec, mode, z_floor_mm, ctx_or_fly);
                obj.executePoseMove(q_target, [x, y, z], pitch, time_sec, mode, z_floor_mm, false);
            end
        end

        function movePosePath(obj, waypoints, opts)
        %MOVEPOSEPATH Execute an Nx4 [x y z pitch] path as one continuous stream.
        %   movePosePath(waypoints)
        %   movePosePath(waypoints, opts)
        %   opts fields:
        %     .speed_mm_s      linear speed target (default 90)
        %     .rot_speed_deg_s pitch speed target (default 90)
        %     .dt              command period seconds (default 0.02)
        %     .z_floor_mm      EE floor safety limit (default 20)
        %     .motion_mode     currently supports continuous mode 2 (default 2)
        %     .smoothing       'smoothstep' or 'linear' (default 'smoothstep')
        %     .final_settle    wait for motion once at end (default true)
        %     .verify_final    verify final pose once at end (default true)
            if nargin < 3 || isempty(opts)
                opts = struct();
            end
            if isempty(waypoints)
                return;
            end
            if size(waypoints, 2) ~= 4
                error('movePosePath expects Nx4 waypoints [x y z pitch].');
            end

            speed_mm_s = 90.0;
            if isfield(opts, 'speed_mm_s') && ~isempty(opts.speed_mm_s)
                speed_mm_s = double(opts.speed_mm_s);
            end
            rot_speed_deg_s = 90.0;
            if isfield(opts, 'rot_speed_deg_s') && ~isempty(opts.rot_speed_deg_s)
                rot_speed_deg_s = double(opts.rot_speed_deg_s);
            end
            dt = 0.02;
            if isfield(opts, 'dt') && ~isempty(opts.dt)
                dt = double(opts.dt);
            end
            z_floor_mm = 20.0;
            if isfield(opts, 'z_floor_mm') && ~isempty(opts.z_floor_mm)
                z_floor_mm = double(opts.z_floor_mm);
            end
            motion_mode = 2;
            if isfield(opts, 'motion_mode') && ~isempty(opts.motion_mode)
                motion_mode = double(opts.motion_mode);
            end
            smoothing = 'smoothstep';
            if isfield(opts, 'smoothing') && ~isempty(opts.smoothing)
                smoothing = char(opts.smoothing);
            end
            final_settle = true;
            if isfield(opts, 'final_settle') && ~isempty(opts.final_settle)
                final_settle = logical(opts.final_settle);
            end
            verify_final = true;
            if isfield(opts, 'verify_final') && ~isempty(opts.verify_final)
                verify_final = logical(opts.verify_final);
            end

            if motion_mode ~= 2
                for i = 1:size(waypoints, 1)
                    wp = waypoints(i, :);
                    obj.moveToPose(wp(1), wp(2), wp(3), wp(4), 0.5, motion_mode, z_floor_mm);
                end
                return;
            end

            q_current = obj.readAngles();
            [T_current, ~] = OpenManipulator.FK(q_current);
            prev_pose = [T_current(1:3, 4)', -(q_current(2) + q_current(3) + q_current(4))];
            q_final = q_current;

            for i = 1:size(waypoints, 1)
                target_pose = double(waypoints(i, :));
                dist_lin = norm(target_pose(1:3) - prev_pose(1:3));
                dist_rot = abs(target_pose(4) - prev_pose(4));
                duration = max([dist_lin / max(speed_mm_s, 1e-6), ...
                                dist_rot / max(rot_speed_deg_s, 1e-6), ...
                                dt]);
                num_steps = max(1, ceil(duration / dt));

                for step = 1:num_steps
                    s = step / num_steps;
                    if strcmpi(smoothing, 'linear')
                        s_interp = s;
                    else
                        s_interp = s * s * (3.0 - 2.0 * s);
                    end
                    pose = (1 - s_interp) * prev_pose + s_interp * target_pose;

                    if pose(3) < z_floor_mm
                        error('Motion Safety Violation: Commanded Z (%.1f mm) < %.1f mm. Aborting.', pose(3), z_floor_mm);
                    end

                    q_interp = OpenManipulator.IK(pose(1), pose(2), pose(3), pose(4));
                    [q_interp, ~] = OpenManipulator.JointLimits.Clamp(q_interp);
                    encoders = zeros(1, 4);
                    for joint_idx = 1:4
                        encoders(joint_idx) = obj.deg2enc(q_interp(joint_idx));
                    end
                    obj.syncWritePositions(encoders);
                    pause(dt);
                    q_final = q_interp;
                end

                prev_pose = target_pose;
            end

            if final_settle
                encoders = zeros(1, 4);
                for joint_idx = 1:4
                    encoders(joint_idx) = obj.deg2enc(q_final(joint_idx));
                end
                for tail = 1:5
                    obj.syncWritePositions(encoders);
                    pause(dt);
                end
                obj.waitForMotion();
            end

            if verify_final
                obj.verifyPose(q_final);
            end
        end

        function moveToAnglesInterpolated(obj, q_target, num_steps, z_floor_mm)
        %MOVETOANGLESINTERPOLATED Smart software interpolation
        %   Uses linear interpolation for large moves, but skips for small moves.
        %   Verifies final position using FK.
        %   Optional z_floor_mm enables EE-only Z floor check.
        
            persistent first_move_done;
            if isempty(first_move_done), first_move_done = false; end
            debug_first = ~first_move_done;

            q_current = obj.readAngles();
            pitch_current = -(q_current(2) + q_current(3) + q_current(4));
            pitch_target  = -(q_target(2) + q_target(3) + q_target(4));

            if debug_first
                fprintf('  [INTERP FIRST] --- First move debug ---\n');
                fprintf('  [INTERP FIRST] start  q=[%.1f %.1f %.1f %.1f]  pitch=%.1f\n', q_current, pitch_current);
                fprintf('  [INTERP FIRST] target q=[%.1f %.1f %.1f %.1f]  pitch=%.1f\n', q_target, pitch_target);
            end

            % Track prior EE Z so floor violations can allow upward recovery.
            prev_z = NaN;
            if nargin >= 4 && ~isempty(z_floor_mm)
                try
                    [T_prev, ~] = OpenManipulator.FK(q_current);
                    prev_z = T_prev(3, 4);
                catch
                    prev_z = NaN;
                end
            end

            if debug_first
                try
                    [T_start, ~] = OpenManipulator.FK(q_current);
                    [T_targ, ~] = OpenManipulator.FK(q_target);
                    fprintf('  [INTERP FIRST] start  FK=[%.1f %.1f %.1f]  target FK=[%.1f %.1f %.1f]\n', ...
                        T_start(1:3,4), T_targ(1:3,4));
                catch
                end
            end

            % Calculate max joint movement
            max_diff = max(abs(q_target - q_current));

            % Determine steps only when caller did not provide num_steps (preserve bridge timing from executePoseMove).
            if nargin < 3
                if max_diff < 10
                    num_steps = 1;
                else
                    num_steps = ceil(max_diff / 5);
                end
            end

            % Ensure at least 1 step
            num_steps = max(1, num_steps);
            if debug_first
                fprintf('  [INTERP FIRST] num_steps=%d  max_joint_diff=%.1f deg\n', num_steps, max_diff);
            end

            % Interpolation Loop
            for step = 1:num_steps
                t = step / num_steps;
                q_interp = (1 - t) * q_current + t * q_target;

                % --- Safety Check ---
                % Perform FK to check for ground collision
                [T_ee, global_transforms] = OpenManipulator.FK(q_interp);

                if nargin >= 4 && ~isempty(z_floor_mm)
                    z_ee = T_ee(3, 4);
                    if z_ee < z_floor_mm
                        % If already below floor, allow only upward recovery motion.
                        if ~isnan(prev_z) && z_ee > (prev_z + 0.01)
                            % Allow recovery toward safe region.
                        else
                            error('Motion Safety Violation: EE < %.1fmm. Aborting.', z_floor_mm);
                        end
                    end
                    prev_z = z_ee;
                else
                    % Legacy safety: Elbow/Wrist + relaxed EE
                    z_elbow = global_transforms(3, 4, 2);
                    z_wrist = global_transforms(3, 4, 3);
                    z_ee    = global_transforms(3, 4, 5);

                    MIN_Z_HEIGHT = 20;
                    MIN_EE_HEIGHT = 5;

                    if z_elbow < MIN_Z_HEIGHT || z_wrist < MIN_Z_HEIGHT || z_ee < MIN_EE_HEIGHT
                        error('Motion Safety Violation: Structure < %dmm or EE < %dmm. Aborting.', MIN_Z_HEIGHT, MIN_EE_HEIGHT);
                    end
                end
                % --------------------

                encoders = zeros(1, 4);
                for i = 1:4
                    encoders(i) = obj.deg2enc(q_interp(i));
                end

                obj.syncWritePositions(encoders);

                if debug_first && (step == 1 || step == num_steps || mod(step, max(1, floor(num_steps/4))) == 0)
                    q_act = obj.readAngles();
                    pitch_act = -(q_act(2) + q_act(3) + q_act(4));
                    try
                        [T_act, ~] = OpenManipulator.FK(q_act);
                        fprintf('  [INTERP FIRST] step %3d/%d t=%.2f  cmd_q=[%.1f %.1f %.1f %.1f]  actual_q=[%.1f %.1f %.1f %.1f]  pitch_act=%.1f  FK_xyz=[%.1f %.1f %.1f]\n', ...
                            step, num_steps, t, q_interp, q_act, pitch_act, T_act(1,4), T_act(2,4), T_act(3,4));
                    catch
                        fprintf('  [INTERP FIRST] step %3d/%d t=%.2f  cmd_q=[%.1f %.1f %.1f %.1f]  actual_q=[%.1f %.1f %.1f %.1f]  pitch_act=%.1f\n', ...
                            step, num_steps, t, q_interp, q_act, pitch_act);
                    end
                end

                pause(0.05);  % 50ms between waypoints
            end

            if debug_first
                q_act = obj.readAngles();
                pitch_act = -(q_act(2) + q_act(3) + q_act(4));
                fprintf('  [INTERP FIRST] after loop (before tail): q_act=[%.1f %.1f %.1f %.1f]  pitch_act=%.1f\n', q_act, pitch_act);
            end

            % Re-send final position (tail) so arm holds at target and doesn't drift/snap back
            encoders = zeros(1, 4);
            for i = 1:4
                encoders(i) = obj.deg2enc(q_target(i));
            end
            for tail = 1:15
                obj.syncWritePositions(encoders);
                pause(0.02);
            end

            if debug_first
                q_act = obj.readAngles();
                pitch_act = -(q_act(2) + q_act(3) + q_act(4));
                fprintf('  [INTERP FIRST] after tail (before wait): q_act=[%.1f %.1f %.1f %.1f]  pitch_act=%.1f\n', q_act, pitch_act);
            end

            % Wait for final position to settle
            obj.waitForMotion();

            if debug_first
                q_act = obj.readAngles();
                pitch_act = -(q_act(2) + q_act(3) + q_act(4));
                fprintf('  [INTERP FIRST] after waitForMotion:       q_act=[%.1f %.1f %.1f %.1f]  pitch_act=%.1f\n', q_act, pitch_act);
                first_move_done = true;
                fprintf('  [INTERP FIRST] --- End first move debug ---\n');
            end

            % Verification
            obj.verifyPose(q_target);
        end

        function executePoseMove(obj, q_target, target_pos, target_pitch, time_sec, mode, z_floor_mm, ctx_or_fly)
        %EXECUTEPOSEMOVE Internal helper to execute a single pose move without rerouting
        %   ctx_or_fly: struct (bridge_ctx with .zones, .final_target_pose, .dynamic_pitch) or logical (is_fly).

            if nargin < 6 || isempty(mode), mode = 1; end
            if nargin < 7 || isempty(z_floor_mm), z_floor_mm = 20.0; end
            if nargin < 8 || isempty(ctx_or_fly)
                ctx_or_fly = false;
            end
            bridge_ctx = [];
            is_fly = false;
            if isstruct(ctx_or_fly)
                bridge_ctx = ctx_or_fly;
            else
                is_fly = logical(ctx_or_fly);
            end
            % When bridge_pick.m calls with preplanned_route=true, use master's timing so script matches master.
            use_bridge_pick_timing = isstruct(ctx_or_fly) && isfield(ctx_or_fly, 'preplanned_route') && ctx_or_fly.preplanned_route;

            debug_jitter = false;  % Set true to log step, t, cmd vs actual (diagnose lag vs oscillation)
            if use_bridge_pick_timing
                log_verify_pose = false;  % Master has no VERIFY logging; avoid timing drift
                dt = 0.05;
            else
                log_verify_pose = true; % Log readAngles+FK (VERIFY START, during segment, VERIFY END). Segment-end jump is from waitForMotion() gap + pause(0.1), not from verify log.
                dt = 0.02;             % 50 Hz (was 0.05). Higher rate = smoother motion for Mode 2/3.
            end

            % Current state (fresh for each segment)
            q_start = obj.readAngles();
            [T_start, ~] = OpenManipulator.FK(q_start);
            start_pos = T_start(1:3, 4);
            % IK uses pitch_deg = -(q2+q3+q4); keep same convention as target_pitch
            start_pitch = -(q_start(2) + q_start(3) + q_start(4));

            dist_lin = norm(target_pos(:) - start_pos(:));
            dist_rot = abs(target_pitch - start_pitch);
            ang_vel_deg_s = 45.0;

            if use_bridge_pick_timing
                % Master duration rule: fixed time_sec for linear moves, same num_steps as master
                if dist_lin > 1e-6
                    dur_lin = max(time_sec, 0.1);
                else
                    dur_lin = 0.0;
                end
                dur_rot = dist_rot / ang_vel_deg_s;
                duration = max([dur_lin, dur_rot, 0.1]);
                num_steps = max(1, ceil(duration / dt));
            else
                % Combine: scale with distance so Cartesian speed is ~constant
                if is_fly
                    cartesian_speed_mm_s = 180.0;   % Fly: high speed so obviously "flying"
                    max_cartesian_speed_mm_s = 300.0;
                else
                    cartesian_speed_mm_s = 70.0;    % Bridge / normal straight-line
                    max_cartesian_speed_mm_s = 120.0;
                end
                if dist_lin > 1e-6
                    dur_lin = dist_lin / cartesian_speed_mm_s;
                    dur_lin = max(dur_lin, 0.15);   % minimum for stability
                    dur_lin = min(dur_lin, time_sec); % cap so one segment doesn't exceed MOVE_TIME
                    dur_lin = max(dur_lin, dist_lin / max_cartesian_speed_mm_s); % no segment faster than max
                else
                    dur_lin = 0.0;
                end
                dur_rot = dist_rot / ang_vel_deg_s;
                duration = max([dur_lin, dur_rot, 0.15]);
                num_steps = max(1, ceil(duration / dt));
            end

            % Skip move when already at target (avoids 0.15s "hold" that can feel like a hiccup before next segment).
            % Do not skip for bridge pick: planner may intend small waypoint steps; master does not skip.
            if ~use_bridge_pick_timing && dist_lin < 1e-6 && dist_rot < 0.5
                return;
            end

            if log_verify_pose
                fprintf('  [VERIFY START] mode=%d target=[%.1f %.1f %.1f] pitch=%.1f | start_q=[%.1f %.1f %.1f %.1f] start_FK=[%.1f %.1f %.1f]\n', ...
                    mode, target_pos(1), target_pos(2), target_pos(3), target_pitch, q_start, start_pos(1), start_pos(2), start_pos(3));
            end

            if mode == 1
                obj.moveToAnglesInterpolated(q_target, num_steps, z_floor_mm);
                if log_verify_pose
                    q_act = obj.readAngles();
                    [T_act, ~] = OpenManipulator.FK(q_act);
                    pos_act = T_act(1:3, 4);
                    err_mm = norm(target_pos(:) - pos_act(:));
                    pitch_act = -(q_act(2) + q_act(3) + q_act(4));
                    fprintf('  [VERIFY END mode=1] actual_q=[%.1f %.1f %.1f %.1f] actual_FK=[%.1f %.1f %.1f] pitch=%.1f | target=[%.1f %.1f %.1f] err_mm=%.2f\n', ...
                        q_act, pos_act(1), pos_act(2), pos_act(3), pitch_act, target_pos(1), target_pos(2), target_pos(3), err_mm);
                end
                return;
            end

            if mode == 2
                % Bridge context: optional dynamic pitch along path (master bridge_pick)
                last_solved_pitch = start_pitch;
                final_target_pose = [target_pos(:)' target_pitch];
                if isstruct(bridge_ctx) && isfield(bridge_ctx, 'final_target_pose')
                    final_target_pose = double(bridge_ctx.final_target_pose(:)');
                end
                use_dynamic_pitch = false;
                if isstruct(bridge_ctx) && isfield(bridge_ctx, 'zones') && ~isempty(bridge_ctx.zones)
                    use_dynamic_pitch = true;
                    if isfield(bridge_ctx, 'dynamic_pitch')
                        use_dynamic_pitch = logical(bridge_ctx.dynamic_pitch);
                    end
                end
                % Task-space linear: interpolate pose, IK each step. Use commanded
                % pose Z for floor check (trajectory we commit to), not FK(q), so
                % small IK/clamp errors do not trigger false safety aborts.
                for step = 1:num_steps
                    s = step / num_steps;
                    % Master uses C1 smooth scaling for segment entry/exit; combine uses linear unless bridge_pick.
                    if use_bridge_pick_timing
                        s_smooth = s * s * (3.0 - 2.0 * s);
                        pose = (1 - s_smooth) * start_pos(:) + s_smooth * target_pos(:);
                        pitch = (1 - s_smooth) * start_pitch + s_smooth * target_pitch;
                    else
                        pose = (1 - s) * start_pos(:) + s * target_pos(:);
                        pitch = (1 - s) * start_pitch + s * target_pitch;
                    end
                    if use_dynamic_pitch
                        dist_to_final = norm(pose(:)' - final_target_pose(1:3));
                        d_far = 160.0;
                        d_near = 25.0;
                        u = (d_far - dist_to_final) / max(1e-6, (d_far - d_near));
                        u = max(0.0, min(1.0, u));
                        proximity = u * u * (3.0 - 2.0 * u);
                        dynamic_target_weight = 25.0 + 220.0 * proximity;
                        dynamic_max_pitch_rate = 1.8 - 0.9 * proximity;
                        desired_pitch = (1.0 - proximity) * last_solved_pitch + proximity * target_pitch;
                        solve_opts = struct();
                        solve_opts.preferred_pitch = desired_pitch;
                        solve_opts.prev_pitch = last_solved_pitch;
                        solve_opts.max_pitch_rate = dynamic_max_pitch_rate;
                        solve_opts.pitch_range = [-90.0, 45.0];
                        solve_opts.terminal_target_pitch = target_pitch;
                        solve_opts.terminal_target_weight = dynamic_target_weight;
                        solve_opts.enforce_terminal_target_if_feasible = false;
                        solve_opts.bridge_proximity_weight = 55.0;
                        solve_opts.bridge_proximity_decay_mm = 12.0;
                        solve_opts.bridge_x_proximity_weight = 95.0;
                        solve_opts.bridge_x_proximity_decay_mm = 9.0;
                        p_opt = OpenManipulator.BridgeAvoidance.SolveOptimalPitch( ...
                            pose(1), pose(2), pose(3), bridge_ctx.zones, solve_opts);
                        if ~isempty(p_opt) && isfinite(p_opt)
                            pitch = p_opt;
                            last_solved_pitch = p_opt;
                        else
                            pitch = last_solved_pitch;
                        end
                    end

                    % Safety: abort only if commanded trajectory goes below floor
                    if pose(3) < z_floor_mm
                        error('Motion Safety Violation: Commanded Z (%.1f mm) < %.1f mm. Aborting.', pose(3), z_floor_mm);
                    end

                    try
                        q_interp = OpenManipulator.IK(pose(1), pose(2), pose(3), pitch);
                    catch ME
                        error('Mode 2 IK failed at step %d (pose Z=%.1f): %s', step, pose(3), ME.message);
                    end
                    [q_interp, ~] = OpenManipulator.JointLimits.Clamp(q_interp);

                    encoders = zeros(1, 4);
                    for i = 1:4
                        encoders(i) = obj.deg2enc(q_interp(i));
                    end
                    obj.syncWritePositions(encoders);
                    if debug_jitter && (mod(step, 5) == 1 || step == num_steps)
                        q_act = obj.readAngles();
                        [T_act, ~] = OpenManipulator.FK(q_act);
                        err_mm = norm(target_pos(:) - T_act(1:3, 4));
                        fprintf('  [jitter dbg] step %d t=%.2f cmd_q=[%.1f %.1f %.1f %.1f] actual_q=[%.1f %.1f %.1f %.1f] err_mm=%.1f\n', ...
                            step, step*dt, q_interp, q_act, err_mm);
                    end
                    if log_verify_pose && (mod(step, 2) == 0 || step == 1 || step == num_steps)
                        q_act = obj.readAngles();
                        [T_act, ~] = OpenManipulator.FK(q_act);
                        pos_act = T_act(1:3, 4);
                        err_mm = norm(target_pos(:) - pos_act(:));
                        fprintf('  [VERIFY mode2] step=%d t=%.2f cmd_q=[%.1f %.1f %.1f %.1f] actual_q=[%.1f %.1f %.1f %.1f] FK=[%.1f %.1f %.1f] err_mm=%.2f\n', ...
                            step, step*dt, q_interp, q_act, pos_act(1), pos_act(2), pos_act(3), err_mm);
                    end
                    pause(dt);
                end

                if use_bridge_pick_timing
                    % Master: no tail; wait then end-of-segment lock then verify
                    obj.waitForMotion();
                    try
                        q_lock = OpenManipulator.IK(target_pos(1), target_pos(2), target_pos(3), target_pitch, 'elbow_up');
                        obj.moveToAnglesInterpolated(q_lock, 1, z_floor_mm);
                    catch
                        % Keep best-effort endpoint if exact lock fails
                    end
                    obj.verifyPose(q_target);
                else
                    % Combine: tail sends then wait then optional VERIFY log then verify
                    encoders = zeros(1, 4);
                    for i = 1:4
                        encoders(i) = obj.deg2enc(q_target(i));
                    end
                    for tail = 1:5
                        obj.syncWritePositions(encoders);
                        pause(dt);
                    end
                    obj.waitForMotion();
                    if log_verify_pose
                        q_act = obj.readAngles();
                        [T_act, ~] = OpenManipulator.FK(q_act);
                        pos_act = T_act(1:3, 4);
                        err_mm = norm(target_pos(:) - pos_act(:));
                        pitch_act = -(q_act(2) + q_act(3) + q_act(4));
                        fprintf('  [VERIFY END mode=2] actual_q=[%.1f %.1f %.1f %.1f] actual_FK=[%.1f %.1f %.1f] pitch=%.1f | target=[%.1f %.1f %.1f] err_mm=%.2f\n', ...
                            q_act, pos_act(1), pos_act(2), pos_act(3), pitch_act, target_pos(1), target_pos(2), target_pos(3), err_mm);
                    end
                    obj.verifyPose(q_target);
                end
                return;
            end

            if mode == 3
                % Jacobian hybrid control
                max_time = max(duration * 1.5, 2.0);
                t = 0.0;
                current_q = q_start;
                jac_final_phase = false;
                jac_final_time = 0.0;
                jac_final_duration = 0.0;
                jac_final_start_q = [];
                jac_final_target_q = [];
                prev_z = T_start(3, 4);

                while t < max_time
                    t = t + dt;

                    if jac_final_phase
                        jac_final_time = jac_final_time + dt;
                        s = jac_final_time / jac_final_duration;
                        s = max(0.0, min(1.0, s));
                        new_q = jac_final_start_q + (jac_final_target_q - jac_final_start_q) * s;
                        if s >= 1.0
                            % Send final position; keep sending for short tail so stream doesn't stop abruptly (reduces jump before VERIFY END)
                            encoders = zeros(1, 4);
                            for i = 1:4
                                encoders(i) = obj.deg2enc(jac_final_target_q(i));
                            end
                            for tail = 1:5
                                obj.syncWritePositions(encoders);
                                pause(dt);
                            end
                            obj.waitForMotion();
                            if log_verify_pose
                                q_act = obj.readAngles();
                                [T_act, ~] = OpenManipulator.FK(q_act);
                                pos_act = T_act(1:3, 4);
                                err_mm = norm(target_pos(:) - pos_act(:));
                                fprintf('  [VERIFY END mode=3] actual_q=[%.1f %.1f %.1f %.1f] actual_FK=[%.1f %.1f %.1f] | target=[%.1f %.1f %.1f] err_mm=%.2f\n', ...
                                    q_act, pos_act(1), pos_act(2), pos_act(3), target_pos(1), target_pos(2), target_pos(3), err_mm);
                            end
                            obj.verifyPose(jac_final_target_q);
                            return;
                        end
                    else
                        [T_curr, ~] = OpenManipulator.FK(current_q);
                        current_pos = T_curr(1:3, 4);
                        % Same convention as target_pitch: pitch_deg = -(q2+q3+q4)
                        current_pitch = -(current_q(2) + current_q(3) + current_q(4));

                        error_pos = target_pos(:) - current_pos(:);
                        error_pitch = target_pitch - current_pitch;

                        handoff_pos_mm = 15.0;
                        handoff_pitch_deg = 8.0;
                        if norm(error_pos) < handoff_pos_mm && abs(error_pitch) < handoff_pitch_deg
                            jac_final_phase = true;
                            jac_final_time = 0.0;
                            jac_final_start_q = current_q;
                            jac_final_target_q = q_target;
                            max_delta = max(abs(jac_final_target_q - jac_final_start_q));
                            final_joint_speed_deg_s = 90.0;
                            jac_final_duration = max(0.12, max_delta / final_joint_speed_deg_s);
                            continue;
                        end

                        % Control gains
                        Kp_pos = 2.0;
                        v_lin = Kp_pos * error_pos;
                        v_norm = norm(v_lin);
                        vel_mag = max(dist_lin / max(duration, 0.1), 10.0);
                        if v_norm > vel_mag && v_norm > 1e-9
                            v_lin = v_lin * (vel_mag / v_norm);
                        end

                        Kp_rot = 2.0;
                        w_pitch_rad = Kp_rot * deg2rad(error_pitch);
                        w_pitch_rad = max(min(w_pitch_rad, deg2rad(90)), deg2rad(-90));

                        % Geometric J is mm/rad and rad/rad: J*q_dot_rad = [v_lin (mm/s); pitch_dot (rad/s)]
                        J = OpenManipulator.GetJacobian(current_q); % 6x4
                        % pitch_deg = -(q2+q3+q4) => pitch_dot_rad = -(q2_dot+q3_dot+q4_dot)_rad
                        J_pitch_row = [0.0, -1.0, -1.0, -1.0];
                        J_task = [J(1:3, :); J_pitch_row];
                        v_task = [v_lin; w_pitch_rad];  % mm/s, mm/s, mm/s, rad/s

                        lambda_val = 0.05;
                        try
                            J_dls = J_task' / (J_task * J_task' + (lambda_val^2) * eye(4));
                            q_dot_rad = J_dls * v_task;  % rad/s
                            q_dot_deg = rad2deg(q_dot_rad);
                            q_dot_deg = max(min(q_dot_deg, 120.0), -120.0);
                            new_q = current_q + q_dot_deg' * dt;
                        catch
                            error('Jacobian Singular: Aborting.');
                        end
                    end

                    % Clamp to joint limits
                    [new_q, ~] = OpenManipulator.JointLimits.Clamp(new_q);

                    % EE Z-floor safety (allow recovery upwards)
                    [T_check, ~] = OpenManipulator.FK(new_q);
                    z_check = T_check(3, 4);
                    if z_check < z_floor_mm
                        if z_check <= (prev_z + 0.01)
                            error('Motion Safety Violation: EE < %.1fmm. Aborting.', z_floor_mm);
                        end
                    end
                    prev_z = z_check;

                    encoders = zeros(1, 4);
                    for i = 1:4
                        encoders(i) = obj.deg2enc(new_q(i));
                    end
                    obj.syncWritePositions(encoders);
                    if debug_jitter && ~jac_final_phase && mod(round(t/dt), 5) == 0
                        q_act = obj.readAngles();
                        [T_act, ~] = OpenManipulator.FK(q_act);
                        err_mm = norm(target_pos(:) - T_act(1:3, 4));
                        fprintf('  [jitter dbg] t=%.2f cmd_q=[%.1f %.1f %.1f %.1f] actual_q=[%.1f %.1f %.1f %.1f] err_mm=%.1f\n', ...
                            t, new_q, q_act, err_mm);
                    end
                    if log_verify_pose && ~jac_final_phase && (mod(round(t/dt), 2) == 0 || t <= dt*1.5)
                        q_act = obj.readAngles();
                        [T_act, ~] = OpenManipulator.FK(q_act);
                        pos_act = T_act(1:3, 4);
                        err_mm = norm(target_pos(:) - pos_act(:));
                        fprintf('  [VERIFY mode3] t=%.2f cmd_q=[%.1f %.1f %.1f %.1f] actual_q=[%.1f %.1f %.1f %.1f] FK=[%.1f %.1f %.1f] err_mm=%.2f\n', ...
                            t, new_q, q_act, pos_act(1), pos_act(2), pos_act(3), err_mm);
                    end
                    pause(dt);

                    current_q = new_q;
                end

                error('Jacobian Motion Timeout.');
            end
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
                id = obj.DXL_IDS(i);
                enc = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, obj.ADDR_PRESENT_POSITION);
                q(i) = obj.enc2deg(enc);
                ok = obj.checkAndLogComResult('readAngles', id);
                if ~ok && ~obj.COM_DEBUG
                    fprintf('[COM] readAngles FAILED at motor ID=%d (comm/rx error above). Robot may have lost connection.\n', id);
                end
            end
        end

        function waitForMotion(obj, timeout)
        %WAITFORMOTION Poll ADDR_MOVING until all motors stopped
        % Short poll/confirm pauses (0.02s) to avoid long gap with no position
        % commands at segment end, which caused visible jump into next segment.
            if nargin < 2
                timeout = 10.0;
            end

            start_time = tic;
            last_log = 0;
            while toc(start_time) < timeout
                elapsed = toc(start_time);
                all_stopped = true;
                moving_ids = [];
                for id = obj.DXL_IDS
                    moving = read1ByteTxRx(obj.port_num, ...
                        obj.PROTOCOL_VERSION, id, obj.ADDR_MOVING);
                    ok = obj.checkAndLogComResult('waitForMotion', id);
                    if ~ok && ~obj.COM_DEBUG
                        fprintf('[COM] waitForMotion read FAILED for ID=%d (possible port drop).\n', id);
                    end
                    if moving == 1
                        all_stopped = false;
                        moving_ids(end+1) = id;
                    end
                end

                if all_stopped
                    % Brief double-check (was 0.1s — reduced to avoid segment-end hitch)
                    pause(0.02);
                    still_stopped = true;
                    for id = obj.DXL_IDS
                        moving = read1ByteTxRx(obj.port_num, ...
                            obj.PROTOCOL_VERSION, id, obj.ADDR_MOVING);
                        obj.checkAndLogComResult('waitForMotion(confirm)', id);
                        if moving == 1
                            still_stopped = false;
                            break;
                        end
                    end
                    if still_stopped
                        return;
                    end
                end

                % Log every 2.5s when waiting so user sees why robot "stops" (or port dropped)
                if ~all_stopped && (elapsed - last_log >= 2.5)
                    fprintf('[COM] waitForMotion t=%.1fs still moving: IDs [%s]\n', elapsed, num2str(moving_ids));
                    last_log = elapsed;
                end
                pause(0.02);
            end
            % Final: report which motors are still moving and do one health check
            moving_ids = [];
            for id = obj.DXL_IDS
                moving = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, id, obj.ADDR_MOVING);
                obj.checkAndLogComResult('waitForMotion(timeout)', id);
                if moving == 1, moving_ids(end+1) = id; end
            end
            warning('Motion timeout after %.1f s. Motors still moving: [%s]. Check COM cable and port.', timeout, num2str(moving_ids));
            obj.checkComHealth(true);
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
            ok = obj.checkAndLogComResult('setGripperPosition', obj.GRIPPER_ID);
            if ~ok && ~obj.COM_DEBUG
                fprintf('[COM] setGripperPosition FAILED (gripper ID=%d). Check cable/port.\n', obj.GRIPPER_ID);
            end
            fprintf('Gripper -> %d%% (encoder %d)\n', pct, enc);
        end

        function pct = readGripperPosition(obj)
        %READGRIPPERPOSITION Read current gripper position as 0–100%
        %   pct = readGripperPosition() — 0 = open, 100 = closed
            enc = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, ...
                obj.GRIPPER_ID, obj.ADDR_PRESENT_POSITION);
            obj.checkAndLogComResult('readGripperPosition', obj.GRIPPER_ID);
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
