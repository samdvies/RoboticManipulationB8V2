classdef BridgeAvoidance
%BRIDGEAVOIDANCE Bridge no-go planning and continuous pitch solver.
%   MATLAB mirror of the Python bridge avoidance utilities.

    methods (Static)
        function zone = NewZone(x_min, x_max, y_min, y_max, z_min, z_max)
            zone = struct('x_min', double(x_min), 'x_max', double(x_max), ...
                          'y_min', double(y_min), 'y_max', double(y_max), ...
                          'z_min', double(z_min), 'z_max', double(z_max));
        end

        function zones = BuildBridgeZones(zone, gap_y, pillar_width_y, deck_thickness)
            if nargin < 2 || isempty(gap_y), gap_y = 50.0; end
            if nargin < 3 || isempty(pillar_width_y), pillar_width_y = 10.0; end
            if nargin < 4 || isempty(deck_thickness), deck_thickness = 10.0; end

            half_gap = gap_y / 2.0;
            pw = pillar_width_y;

            left_pillar = OpenManipulator.BridgeAvoidance.NewZone( ...
                zone.x_min, zone.x_max, -half_gap - pw, -half_gap, zone.z_min, zone.z_max);
            right_pillar = OpenManipulator.BridgeAvoidance.NewZone( ...
                zone.x_min, zone.x_max, half_gap, half_gap + pw, zone.z_min, zone.z_max);
            deck = OpenManipulator.BridgeAvoidance.NewZone( ...
                zone.x_min, zone.x_max, -half_gap - pw, half_gap + pw, zone.z_max - deck_thickness, zone.z_max);

            zones = [left_pillar, right_pillar, deck];
        end

        function inside = ContainsPoint(zone, point_xyz, padding_mm)
            if nargin < 3 || isempty(padding_mm), padding_mm = 0.0; end
            p = max(0.0, double(padding_mm));
            x = double(point_xyz(1)); y = double(point_xyz(2)); z = double(point_xyz(3));
            inside = ((zone.x_min - p) <= x && x <= (zone.x_max + p)) && ...
                     ((zone.y_min - p) <= y && y <= (zone.y_max + p)) && ...
                     ((zone.z_min - p) <= z && z <= (zone.z_max + p));
        end

        function hit = SegmentIntersectsNoGoZone(start_xyz, end_xyz, zones, samples, padding_mm)
            if nargin < 4 || isempty(samples), samples = 64; end
            if nargin < 5 || isempty(padding_mm), padding_mm = 0.0; end
            if ~isstruct(zones)
                hit = false;
                return;
            end
            n = max(2, round(samples));
            s = double(start_xyz(:)');
            e = double(end_xyz(:)');
            for i = 0:n
                t = i / n;
                p = s + (e - s) * t;
                for k = 1:numel(zones)
                    if OpenManipulator.BridgeAvoidance.ContainsPoint(zones(k), p, padding_mm)
                        hit = true;
                        return;
                    end
                end
            end
            hit = false;
        end

        function waypoints = PlanBridgeSafeWaypoints(start_pose, target_pose, zone, zones, opts)
            if nargin < 4 || isempty(zones), zones = zone; end
            if nargin < 5 || isempty(opts), opts = struct(); end

            pitch_tolerance_deg = OpenManipulator.BridgeAvoidance.getOpt(opts, 'pitch_tolerance_deg', 5.0);
            vertical_clearance_mm = OpenManipulator.BridgeAvoidance.getOpt(opts, 'vertical_clearance_mm', 30.0);
            samples = OpenManipulator.BridgeAvoidance.getOpt(opts, 'samples', 20);

            start = double(start_pose(:)');
            target = double(target_pose(:)');

            start_under = start(3) < zone.z_max;
            target_under = target(3) < zone.z_max;
            target_horiz = abs(target(4)) <= abs(pitch_tolerance_deg);
            approach_pitch = 0.0;

            % Keep staging farther in front of the bridge (smaller X) to avoid
            % early inward diagonals that clip bridge geometry on hardware.
            approach_x = OpenManipulator.BridgeAvoidance.findSafeApproachX(target(3), target(2), approach_pitch, zones, 80.0, 30.0, zone.x_min);
            min_safe_z = OpenManipulator.BridgeAvoidance.findMinSafeZ(approach_x, target(2), approach_pitch, zones, 10.0, zone.z_max + 80.0);
            safe_z = max(min_safe_z, zone.z_max) + abs(vertical_clearance_mm);

            routeIsSafe = @(route) OpenManipulator.BridgeAvoidance.routeIsSafe(start, route, zones, samples);

            if start_under && ~target_under
                empty_zone = OpenManipulator.BridgeAvoidance.NewZone(0, 0, 0, 0, 0, 0);
                exit_x = min([approach_x, zone.x_min - 40.0]);
                min_reachable_z = OpenManipulator.BridgeAvoidance.findMinSafeZ(exit_x, start(2), approach_pitch, empty_zone, 10.0, safe_z);
                drop_z = max(start(3), min_reachable_z + 5.0);

                wp_slide_out = [exit_x, start(2), drop_z, start(4)];
                wp_lift = [exit_x, start(2), safe_z, target(4)];
                route_a = OpenManipulator.BridgeAvoidance.dedupeWaypoints([wp_slide_out; wp_lift; target]);
                if routeIsSafe(route_a), waypoints = route_a; return; end

                wp_over = [target(1), target(2), safe_z, target(4)];
                route_b = OpenManipulator.BridgeAvoidance.dedupeWaypoints([wp_slide_out; wp_lift; wp_over; target]);
                if routeIsSafe(route_b), waypoints = route_b; return; end

                wp_up = [start(1), start(2), safe_z, start(4)];
                route_c = OpenManipulator.BridgeAvoidance.dedupeWaypoints([wp_up; wp_over; target]);
                if routeIsSafe(route_c), waypoints = route_c; return; end

                waypoints = target;
                return;
            end

            if start_under && target_under
                exit_x = min([zone.x_min - 40.0, start(1), target(1)]);
                wp_slide_out = [exit_x, start(2), start(3), start(4)];
                wp_adjust = [exit_x, target(2), target(3), target(4)];
                route_a = OpenManipulator.BridgeAvoidance.dedupeWaypoints([wp_slide_out; wp_adjust; target]);
                if routeIsSafe(route_a), waypoints = route_a; return; end

                wp_xy = [target(1), target(2), start(3), target(4)];
                route_b = OpenManipulator.BridgeAvoidance.dedupeWaypoints([wp_xy; target]);
                if routeIsSafe(route_b), waypoints = route_b; return; end

                waypoints = route_a;
                return;
            end

            if target_under && target_horiz
                % Entry mirrors exit: plan reverse (target->start) using the same
                % first-feasible staged logic, then reverse waypoints back.
                rev = OpenManipulator.BridgeAvoidance.PlanBridgeSafeWaypoints(target, start, zone, zones, opts);
                waypoints = OpenManipulator.BridgeAvoidance.dedupeWaypoints(flipud(rev));
                if isempty(waypoints) || any(abs(waypoints(end,:) - target) > 1e-6)
                    waypoints = OpenManipulator.BridgeAvoidance.dedupeWaypoints([waypoints; target]);
                end
                return;
            end

            if ~OpenManipulator.BridgeAvoidance.checkPathSegment(start, target, zones, samples)
                waypoints = target;
                return;
            end

            wp_up = [start(1), start(2), safe_z, start(4)];
            wp_over = [target(1), target(2), safe_z, target(4)];
            ok = ~OpenManipulator.BridgeAvoidance.checkPathSegment(start, wp_up, zones, samples) && ...
                 ~OpenManipulator.BridgeAvoidance.checkPathSegment(wp_up, wp_over, zones, samples) && ...
                 ~OpenManipulator.BridgeAvoidance.checkPathSegment(wp_over, target, zones, samples);
            if ok
                waypoints = OpenManipulator.BridgeAvoidance.dedupeWaypoints([wp_up; wp_over; target]);
            else
                waypoints = target;
            end
        end

        function jaw = ClampGripperWidthForBridge(jaw_width_mm, min_width_mm, max_width_mm)
            if nargin < 2 || isempty(min_width_mm), min_width_mm = 25.0; end
            if nargin < 3 || isempty(max_width_mm), max_width_mm = 50.0; end
            lo = min(min_width_mm, max_width_mm);
            hi = max(min_width_mm, max_width_mm);
            jaw = min(max(double(jaw_width_mm), lo), hi);
        end

        function best_pitch = SolveOptimalPitch(x, y, z, zones, opts)
            if nargin < 5 || isempty(opts), opts = struct(); end
            preferred_pitch = OpenManipulator.BridgeAvoidance.getOpt(opts, 'preferred_pitch', 0.0);
            prev_pitch = OpenManipulator.BridgeAvoidance.getOpt(opts, 'prev_pitch', []);
            max_pitch_rate = OpenManipulator.BridgeAvoidance.getOpt(opts, 'max_pitch_rate', 4.0);
            pitch_range = OpenManipulator.BridgeAvoidance.getOpt(opts, 'pitch_range', [-90.0, 45.0]);
            margin_deg = OpenManipulator.BridgeAvoidance.getOpt(opts, 'margin_deg', 5.0);
            terminal_target_pitch = OpenManipulator.BridgeAvoidance.getOpt(opts, 'terminal_target_pitch', []);
            terminal_target_weight = OpenManipulator.BridgeAvoidance.getOpt(opts, 'terminal_target_weight', 200.0);
            enforce_terminal_target_if_feasible = OpenManipulator.BridgeAvoidance.getOpt(opts, 'enforce_terminal_target_if_feasible', false);
            bridge_proximity_weight = OpenManipulator.BridgeAvoidance.getOpt(opts, 'bridge_proximity_weight', 35.0);
            bridge_proximity_decay_mm = OpenManipulator.BridgeAvoidance.getOpt(opts, 'bridge_proximity_decay_mm', 12.0);
            bridge_proximity_samples = OpenManipulator.BridgeAvoidance.getOpt(opts, 'bridge_proximity_samples', 4);
            bridge_x_proximity_weight = OpenManipulator.BridgeAvoidance.getOpt(opts, 'bridge_x_proximity_weight', 55.0);
            bridge_x_proximity_decay_mm = OpenManipulator.BridgeAvoidance.getOpt(opts, 'bridge_x_proximity_decay_mm', 10.0);

            lim = OpenManipulator.JointLimits.GetLimits();
            vmin = lim(:,1)' + margin_deg;
            vmax = lim(:,2)' - margin_deg;

            p_min = floor(pitch_range(1));
            p_max = ceil(pitch_range(2));

            if ~isempty(terminal_target_pitch) && enforce_terminal_target_if_feasible
                p0 = double(terminal_target_pitch);
                if p0 >= p_min && p0 <= p_max && OpenManipulator.BridgeAvoidance.pitchFeasible(x,y,z,p0,zones,vmin,vmax)
                    best_pitch = p0;
                    return;
                end
            end

            best_pitch = [];
            min_cost = inf;
            for p = p_min:p_max
                c = OpenManipulator.BridgeAvoidance.evaluatePitchCost(x, y, z, double(p), zones, vmin, vmax, ...
                    preferred_pitch, prev_pitch, max_pitch_rate, terminal_target_pitch, terminal_target_weight, ...
                    bridge_proximity_weight, bridge_proximity_decay_mm, bridge_proximity_samples, ...
                    bridge_x_proximity_weight, bridge_x_proximity_decay_mm);
                if c < min_cost
                    min_cost = c;
                    best_pitch = double(p);
                end
            end

            cand = [prev_pitch, preferred_pitch];
            for i = 1:numel(cand)
                p = cand(i);
                if isempty(p), continue; end
                if ~isempty(prev_pitch) && abs(double(p) - double(prev_pitch)) > (1.5 * max_pitch_rate)
                    continue;
                end
                c = OpenManipulator.BridgeAvoidance.evaluatePitchCost(x, y, z, double(p), zones, vmin, vmax, ...
                    preferred_pitch, prev_pitch, max_pitch_rate, terminal_target_pitch, terminal_target_weight, ...
                    bridge_proximity_weight, bridge_proximity_decay_mm, bridge_proximity_samples, ...
                    bridge_x_proximity_weight, bridge_x_proximity_decay_mm);
                if c < min_cost
                    min_cost = c;
                    best_pitch = double(p);
                end
            end
        end
    end

    methods (Static, Access = private)
        function v = getOpt(opts, key, default)
            if isstruct(opts) && isfield(opts, key)
                v = opts.(key);
            else
                v = default;
            end
        end

        function route = dedupeWaypoints(route)
            if isempty(route), return; end
            out = route(1, :);
            for i = 2:size(route, 1)
                if any(abs(route(i,:) - out(end,:)) > 1e-6)
                    out = [out; route(i,:)]; %#ok<AGROW>
                end
            end
            route = out;
        end

        function unsafe = checkPathSegment(start_pose, end_pose, zones, samples)
            unsafe = false;
            n = max(2, round(samples));
            for i = 0:n
                t = i / n;
                p = start_pose(1:3) + (end_pose(1:3) - start_pose(1:3)) * t;
                pitch = start_pose(4) + (end_pose(4) - start_pose(4)) * t;
                try
                    q = OpenManipulator.IK(p(1), p(2), p(3), pitch, 'elbow_up', false);
                    [valid, ~] = OpenManipulator.JointLimits.Validate(q);
                    if ~valid
                        unsafe = true;
                        return;
                    end
                    [~, tf] = OpenManipulator.FK(q);
                    for j = 1:(size(tf,3)-1)
                        p_start = tf(1:3,4,j)';
                        p_end = tf(1:3,4,j+1)';
                        if OpenManipulator.BridgeAvoidance.SegmentIntersectsNoGoZone(p_start, p_end, zones, 10, 0.0)
                            unsafe = true;
                            return;
                        end
                    end
                catch
                    unsafe = true;
                    return;
                end
            end
        end

        function ok = routeIsSafe(start_wp, route_wps, zones, samples)
            ok = true;
            prev = start_wp;
            for i = 1:size(route_wps,1)
                curr = route_wps(i,:);
                if OpenManipulator.BridgeAvoidance.checkPathSegment(prev, curr, zones, samples)
                    ok = false;
                    return;
                end
                prev = curr;
            end
        end

        function safe = armSafeAt(x,y,z,pitch,zones)
            safe = false;
            try
                q = OpenManipulator.IK(x,y,z,pitch,'elbow_up',false);
                [valid, ~] = OpenManipulator.JointLimits.Validate(q);
                if ~valid, return; end
                [~, tf] = OpenManipulator.FK(q);
                for j = 1:(size(tf,3)-1)
                    p_start = tf(1:3,4,j)';
                    p_end = tf(1:3,4,j+1)';
                    if OpenManipulator.BridgeAvoidance.SegmentIntersectsNoGoZone(p_start, p_end, zones, 10, 0.0)
                        return;
                    end
                end
                safe = true;
            catch
                safe = false;
            end
        end

        function z_safe = findMinSafeZ(x, y, pitch, zones, z_low, z_high)
            if nargin < 6 || isempty(z_low), z_low = 10.0; end
            if nargin < 7 || isempty(z_high), z_high = 300.0; end
            tol = 2.0;
            if ~OpenManipulator.BridgeAvoidance.armSafeAt(x,y,z_high,pitch,zones)
                z_safe = z_high;
                return;
            end
            lo = z_low; hi = z_high;
            for i = 1:20
                if (hi - lo) < tol, break; end
                mid = (lo + hi) / 2.0;
                if OpenManipulator.BridgeAvoidance.armSafeAt(x,y,mid,pitch,zones)
                    hi = mid;
                else
                    lo = mid;
                end
            end
            z_safe = hi;
        end

        function x_best = findSafeApproachX(target_z, target_y, pitch, zones, x_min, x_max_offset, zone_x_min)
            if nargin < 5 || isempty(x_min), x_min = 50.0; end
            if nargin < 6 || isempty(x_max_offset), x_max_offset = 10.0; end
            x_right = zone_x_min - x_max_offset;
            x_left = x_min;
            drop_z_top = 90.0;
            lo = x_left; hi = x_right;
            x_best = x_left;
            for i = 1:20
                if (hi - lo) < 2.0, break; end
                mid = (lo + hi) / 2.0;
                drop_start = [mid, target_y, drop_z_top, pitch];
                drop_end = [mid, target_y, target_z, pitch];
                drop_ok = ~OpenManipulator.BridgeAvoidance.checkPathSegment(drop_start, drop_end, zones, 20);
                if drop_ok
                    x_best = mid;
                    lo = mid;
                else
                    hi = mid;
                end
            end
        end

        function [dx, dy, dz] = pointToZoneOutsideDeltas(point_xyz, zone)
            px = point_xyz(1); py = point_xyz(2); pz = point_xyz(3);
            dx = 0.0; dy = 0.0; dz = 0.0;
            if px < zone.x_min, dx = zone.x_min - px; elseif px > zone.x_max, dx = px - zone.x_max; end
            if py < zone.y_min, dy = zone.y_min - py; elseif py > zone.y_max, dy = py - zone.y_max; end
            if pz < zone.z_min, dz = zone.z_min - pz; elseif pz > zone.z_max, dz = pz - zone.z_max; end
        end

        function feasible = pitchFeasible(x,y,z,p,zones,vmin,vmax)
            feasible = false;
            try
                q = OpenManipulator.IK(x,y,z,p,'elbow_up',false);
            catch
                return;
            end
            if any(q <= vmin) || any(q >= vmax), return; end
            [~, tf] = OpenManipulator.FK(q);
            for j = 1:(size(tf,3)-1)
                p_start = tf(1:3,4,j)';
                p_end = tf(1:3,4,j+1)';
                if OpenManipulator.BridgeAvoidance.SegmentIntersectsNoGoZone(p_start, p_end, zones, 5, 2.0)
                    return;
                end
            end
            feasible = true;
        end

        function cost = evaluatePitchCost(x,y,z,p,zones,vmin,vmax,preferred_pitch,prev_pitch,max_pitch_rate,terminal_target_pitch,terminal_target_weight,bridge_proximity_weight,bridge_proximity_decay_mm,bridge_proximity_samples,bridge_x_proximity_weight,bridge_x_proximity_decay_mm)
            try
                q = OpenManipulator.IK(x,y,z,p,'elbow_up',false);
            catch
                cost = inf;
                return;
            end

            % Joint margin hard check + limit cost
            if any(q <= vmin) || any(q >= vmax)
                cost = inf;
                return;
            end
            cost_limits = 0.0;
            for i = 1:numel(q)
                center = (vmax(i) + vmin(i)) / 2.0;
                range_half = (vmax(i) - vmin(i)) / 2.0;
                norm_dist = abs(q(i) - center) / max(1e-6, range_half);
                cost_limits = cost_limits + (exp(norm_dist * 5.0) - 1.0);
            end

            [~, tf] = OpenManipulator.FK(q);
            % Hard collision + soft proximity
            cost_proximity = 0.0;
            decay = max(1e-6, bridge_proximity_decay_mm);
            x_decay = max(1e-6, bridge_x_proximity_decay_mm);
            samples = max(2, round(bridge_proximity_samples));
            total_dist = 0.0;
            total_x = 0.0;
            count = 0;

            for j = 1:(size(tf,3)-1)
                p0 = tf(1:3,4,j)';
                p1 = tf(1:3,4,j+1)';

                if OpenManipulator.BridgeAvoidance.SegmentIntersectsNoGoZone(p0, p1, zones, 5, 2.0)
                    cost = inf;
                    return;
                end

                for i = 0:samples
                    t = i / samples;
                    pxyz = p0 + (p1 - p0) * t;
                    min_dist = inf;
                    min_dx = inf;
                    for k = 1:numel(zones)
                        [dx, dy, dz] = OpenManipulator.BridgeAvoidance.pointToZoneOutsideDeltas(pxyz, zones(k));
                        d = sqrt(dx*dx + dy*dy + dz*dz);
                        min_dist = min(min_dist, d);
                        min_dx = min(min_dx, dx);
                    end
                    total_dist = total_dist + exp(-min_dist / decay);
                    total_x = total_x + exp(-min_dx / x_decay);
                    count = count + 1;
                end
            end

            if count > 0
                mean_dist = total_dist / count;
                mean_x = total_x / count;
                cost_proximity = bridge_proximity_weight * mean_dist + bridge_x_proximity_weight * mean_x;
            end

            % Smoothness and preference
            cost_smooth = 0.0;
            if ~isempty(prev_pitch)
                delta = abs(p - prev_pitch);
                if delta > max_pitch_rate
                    excess = delta - max_pitch_rate;
                    cost_smooth = cost_smooth + 140.0 * (excess ^ 2);
                end
                cost_smooth = cost_smooth + 3.0 * delta;
            end
            cost_smooth = cost_smooth + 0.5 * abs(p - preferred_pitch);

            cost_terminal = 0.0;
            if ~isempty(terminal_target_pitch)
                cost_terminal = abs(p - terminal_target_pitch) * terminal_target_weight;
            end

            cost = cost_limits + cost_proximity + cost_smooth + cost_terminal;
        end

    end
end
