classdef BlueBoat
    %Simple 2D surface vessel simulator producing:
    %  - ground_truth_trajectory: true trajectory it followed
    %  - est_trajectory: simulated measurements with covariance
    %       - Structure as if we had GPS + IMU onboard

    properties
        name (1,1) string = "BlueBoat"

        % --- Initial state ---
        x0 (1,1) double = 0
        y0 (1,1) double = 0
        theta0 (1,1) double = 0

        % --- Simulation timing ---
        dt (1,1) double {mustBePositive} = 0.1

        % --- Simple motion constraints ---
        nominal_speed (1,1) double {mustBeNonnegative} = 1.0  % nominal speed [m/s]
        yaw_rate_max (1,1) double {mustBeNonnegative} = 0.6   % max yaw rate [rad/s]
        accel_max (1,1) double {mustBeNonnegative} = 0.5      % max accel [m/s^2] (optional)

        % --- Waypoint following ---
        wp_accept_radius (1,1) double {mustBePositive} = 1.0  % [m]
        heading_kp (1,1) double {mustBePositive} = 1.5        % P controller to waypoint
        slow_down_radius (1,1) double {mustBePositive} = 5.0  % start slowing near wp

        % --- Sensor rates ---
        gps_period (1,1) double {mustBePositive} = 1.0        % seconds
        heading_period (1,1) double {mustBePositive} = 0.2    % seconds

        % --- Sensor noise (1-sigma) ---
        gps_sigma_xy (1,1) double {mustBePositive} = 0.8         % [m]
        heading_sigma (1,1) double {mustBePositive} = deg2rad(3) % [rad]

        % --- IMU/odometry noise used in prediction model ---
        odom_sigma_v (1,1) double {mustBeNonnegative} = 0.15  % [m/s]
        odom_sigma_w (1,1) double {mustBeNonnegative} = 0.05  % [rad/s]

        bias_v (1,1) double = 0.0    % [m/s]
        bias_w (1,1) double = 0.0    % [rad/s]

        % Random seed control
        rng_seed (1,1) double = 1
    end

    methods
        function obj = BlueBoat(varargin)
            % Name-value constructor
            if isempty(varargin)
                return;
            end

            input_parser = inputParser;
            input_parser.FunctionName = "BlueBoat";

            % --- Identity / timing / init ---
            addParameter(input_parser, "name", obj.name);
            addParameter(input_parser, "dt", obj.dt);
            addParameter(input_parser, "x0", obj.x0);
            addParameter(input_parser, "y0", obj.y0);
            addParameter(input_parser, "theta0", obj.theta0);

            % --- Motion constraints ---
            addParameter(input_parser, "nominal_speed", obj.nominal_speed);
            addParameter(input_parser, "yaw_rate_max", obj.yaw_rate_max);
            addParameter(input_parser, "accel_max", obj.accel_max);

            % --- Waypoint following ---
            addParameter(input_parser, "wp_accept_radius", obj.wp_accept_radius);
            addParameter(input_parser, "heading_kp", obj.heading_kp);
            addParameter(input_parser, "slow_down_radius", obj.slow_down_radius);

            % --- Sensor rates ---
            addParameter(input_parser, "gps_period", obj.gps_period);
            addParameter(input_parser, "heading_period", obj.heading_period);

            % --- Sensor noise ---
            addParameter(input_parser, "gps_sigma_xy", obj.gps_sigma_xy);
            addParameter(input_parser, "heading_sigma", obj.heading_sigma);

            % --- Odom noise ---
            addParameter(input_parser, "odom_sigma_v", obj.odom_sigma_v);
            addParameter(input_parser, "odom_sigma_w", obj.odom_sigma_w);

            % --- Biases ---
            addParameter(input_parser, "bias_v", obj.bias_v);
            addParameter(input_parser, "bias_w", obj.bias_w);

            % --- RNG ---
            addParameter(input_parser, "rng_seed", obj.rng_seed);

            parse(input_parser, varargin{:});
            parsed_results = input_parser.Results;

            obj.name = parsed_results.name;

            obj.dt = parsed_results.dt;
            obj.x0 = parsed_results.x0;
            obj.y0 = parsed_results.y0;
            obj.theta0 = wrap_to_pi(parsed_results.theta0);

            obj.nominal_speed = parsed_results.nominal_speed;
            obj.yaw_rate_max  = parsed_results.yaw_rate_max;
            obj.accel_max     = parsed_results.accel_max;

            obj.wp_accept_radius = parsed_results.wp_accept_radius;
            obj.heading_kp = parsed_results.heading_kp;
            obj.slow_down_radius = parsed_results.slow_down_radius;

            obj.gps_period = parsed_results.gps_period;
            obj.heading_period = parsed_results.heading_period;

            obj.gps_sigma_xy = parsed_results.gps_sigma_xy;
            obj.heading_sigma = parsed_results.heading_sigma;

            obj.odom_sigma_v = parsed_results.odom_sigma_v;
            obj.odom_sigma_w = parsed_results.odom_sigma_w;

            obj.bias_v = parsed_results.bias_v;
            obj.bias_w = parsed_results.bias_w;

            obj.rng_seed = parsed_results.rng_seed;
        end

        function sim = simulate(obj, waypoints, T_total, varargin)
            %Run waypoint-following sim for T_total seconds.
            %
            % Inputs:
            %   waypoints: Nx2 [x y]
            %   T_total:   scalar seconds
            %
            % Optional Name-Value:
            %   "start_time" : datetime (default: NaT)
            %   "P0"         : initial covariance (default diag)
            %   "truth_process_noise" : [sigma_v_true, sigma_w_true] (default 0)

            input_parser = inputParser;
            addParameter(input_parser, "start_time", NaT, @(d) isdatetime(d) && isscalar(d));
            addParameter(input_parser, "P0", diag([1, 1, deg2rad(5)].^2), @(cov) isnumeric(cov) && isequal(size(cov), [3 3]));
            addParameter(input_parser, "truth_process_noise", [0.0, 0.0], @(x) isnumeric(x) && numel(x) == 2);
            parse(input_parser, varargin{:});

            start_time = input_parser.Results.start_time;
            covariance_estimate = input_parser.Results.P0;
            truth_process_noise = input_parser.Results.truth_process_noise(:);

            rng(obj.rng_seed);

            num_steps = floor(T_total / obj.dt) + 1;
            time_sec = (0:num_steps-1)' * obj.dt;

            ground_truth_trajectory(num_steps, 1) = Pose2();
            est_trajectory(num_steps, 1) = Pose2();

            ground_truth_trajectory(1) = Pose2(obj.x0, obj.y0, obj.theta0, "timestamp", start_time, "covariance", eye(3));
            est_trajectory(1) = Pose2(obj.x0, obj.y0, obj.theta0, "timestamp", start_time, "covariance", covariance_estimate);

            covariance_history = zeros(3, 3, num_steps);
            covariance_history(:, :, 1) = covariance_estimate;

            speed_cmd_history = zeros(num_steps, 1);
            yaw_rate_cmd_history = zeros(num_steps, 1);
            waypoint_index_history = zeros(num_steps, 1);

            waypoint_index = 1;
            next_gps_time_sec = 0.0;
            next_heading_time_sec = 0.0;

            for step_index = 2:num_steps
                current_time_sec = time_sec(step_index);

                % --- Waypoint logic ---
                [waypoint_index, speed_cmd, yaw_rate_cmd] = obj.compute_control( ...
                    ground_truth_trajectory(step_index-1), waypoints, waypoint_index);

                speed_cmd_history(step_index) = speed_cmd;
                yaw_rate_cmd_history(step_index) = yaw_rate_cmd;
                waypoint_index_history(step_index) = waypoint_index;

                % --- Process noise ---
                speed_true = speed_cmd + truth_process_noise(1) * randn();
                yaw_rate_true = yaw_rate_cmd + truth_process_noise(2) * randn();

                ground_truth_trajectory(step_index) = obj.integrate_pose( ...
                    ground_truth_trajectory(step_index-1), speed_true, yaw_rate_true, start_time, current_time_sec);

                % --- Propagate estimate ---
                speed_meas = speed_cmd + obj.bias_v + obj.odom_sigma_v * randn();
                yaw_rate_meas = yaw_rate_cmd + obj.bias_w + obj.odom_sigma_w * randn();

                [predicted_state, state_jacobian, control_jacobian] = obj.motion_model( ...
                    est_trajectory(step_index-1), speed_meas, yaw_rate_meas, obj.dt);

                process_noise_cov = diag([obj.odom_sigma_v^2, obj.odom_sigma_w^2]);
                covariance_estimate = state_jacobian * covariance_estimate * state_jacobian.' + ...
                                      control_jacobian * process_noise_cov * control_jacobian.';

                est_trajectory(step_index) = Pose2( ...
                    predicted_state(1), predicted_state(2), predicted_state(3), ...
                    "timestamp", obj.time_from_start(start_time, current_time_sec), ...
                    "covariance", covariance_estimate);

                % --- Apply GPS update (x,y) ---
                if current_time_sec + 1e-12 >= next_gps_time_sec
                    gps_measurement = [ground_truth_trajectory(step_index).x; ground_truth_trajectory(step_index).y] + ...
                                      obj.gps_sigma_xy * randn(2, 1);

                    [est_trajectory(step_index), covariance_estimate] = obj.meas_update_xy( ...
                        est_trajectory(step_index), covariance_estimate, gps_measurement, obj.gps_sigma_xy^2);

                    next_gps_time_sec = next_gps_time_sec + obj.gps_period;
                end

                % --- Apply heading update (theta) ---
                if current_time_sec + 1e-12 >= next_heading_time_sec
                    heading_measurement = wrap_to_pi(ground_truth_trajectory(step_index).theta + obj.heading_sigma * randn());

                    [est_trajectory(step_index), covariance_estimate] = obj.meas_update_theta( ...
                        est_trajectory(step_index), covariance_estimate, heading_measurement, obj.heading_sigma^2);

                    next_heading_time_sec = next_heading_time_sec + obj.heading_period;
                end

                est_trajectory(step_index).covariance = covariance_estimate;
                covariance_history(:, :, step_index) = covariance_estimate;
            end

            sim = struct();
            sim.time_sec = time_sec;
            sim.ground_truth_trajectory = ground_truth_trajectory;
            sim.est_trajectory = est_trajectory;

            sim.covariance_history = covariance_history;
            sim.speed_cmd_history = speed_cmd_history;
            sim.yaw_rate_cmd_history = yaw_rate_cmd_history;
            sim.waypoint_index_history = waypoint_index_history;

            sim.ground_truth_xytheta = [[ground_truth_trajectory.x].', [ground_truth_trajectory.y].', [ground_truth_trajectory.theta].'];
            sim.est_xytheta = [[est_trajectory.x].', [est_trajectory.y].', [est_trajectory.theta].'];
        end
    end

    methods (Access = private)
        function [waypoint_index, speed_cmd, yaw_rate_cmd] = compute_control(obj, pose, waypoints, waypoint_index)
            % Basic waypoint follower: steer towards current waypoint
            if waypoint_index > size(waypoints, 1)
                speed_cmd = 0;
                yaw_rate_cmd = 0;
                return;
            end

            waypoint_xy = waypoints(waypoint_index, :).';
            delta_x = waypoint_xy(1) - pose.x;
            delta_y = waypoint_xy(2) - pose.y;
            distance_to_waypoint = hypot(delta_x, delta_y);

            if distance_to_waypoint < obj.wp_accept_radius
                waypoint_index = waypoint_index + 1;
                if waypoint_index > size(waypoints, 1)
                    speed_cmd = 0;
                    yaw_rate_cmd = 0;
                    return;
                end

                waypoint_xy = waypoints(waypoint_index, :).';
                delta_x = waypoint_xy(1) - pose.x;
                delta_y = waypoint_xy(2) - pose.y;
                distance_to_waypoint = hypot(delta_x, delta_y);
            end

            desired_heading = atan2(delta_y, delta_x);
            heading_error = wrap_to_pi(desired_heading - pose.theta);

            yaw_rate_cmd = obj.heading_kp * heading_error;
            yaw_rate_cmd = max(-obj.yaw_rate_max, min(obj.yaw_rate_max, yaw_rate_cmd));

            speed_cmd = obj.nominal_speed;
            if distance_to_waypoint < obj.slow_down_radius
                speed_cmd = obj.nominal_speed * (distance_to_waypoint / obj.slow_down_radius);
            end
        end

        function pose_next = integrate_pose(obj, pose_prev, speed, yaw_rate, start_time, current_time_sec)
            previous_heading = pose_prev.theta;

            next_heading = wrap_to_pi(previous_heading + yaw_rate * obj.dt);
            next_x = pose_prev.x + speed * cos(previous_heading) * obj.dt;
            next_y = pose_prev.y + speed * sin(previous_heading) * obj.dt;

            pose_next = Pose2(next_x, next_y, next_heading, ...
                "timestamp", obj.time_from_start(start_time, current_time_sec), ...
                "covariance", eye(3));
        end

        function [predicted_state, state_jacobian, control_jacobian] = motion_model(obj, pose_prev, speed, yaw_rate, dt)
            previous_heading = pose_prev.theta;

            predicted_state = zeros(3, 1);
            predicted_state(1) = pose_prev.x + speed * cos(previous_heading) * dt;
            predicted_state(2) = pose_prev.y + speed * sin(previous_heading) * dt;
            predicted_state(3) = wrap_to_pi(previous_heading + yaw_rate * dt);

            state_jacobian = eye(3);
            state_jacobian(1, 3) = -speed * sin(previous_heading) * dt;
            state_jacobian(2, 3) =  speed * cos(previous_heading) * dt;

            control_jacobian = zeros(3, 2);
            control_jacobian(1, 1) = cos(previous_heading) * dt;
            control_jacobian(2, 1) = sin(previous_heading) * dt;
            control_jacobian(3, 2) = dt;
        end

        function [pose_out, covariance_out] = meas_update_xy(obj, pose_in, covariance_in, gps_measurement_xy, gps_variance_xy)
            measurement_jacobian = [1 0 0;
                                    0 1 0];
            measurement_cov = gps_variance_xy * eye(2);

            state_estimate = [pose_in.x; pose_in.y; pose_in.theta];
            measurement_residual = gps_measurement_xy - measurement_jacobian * state_estimate;

            innovation_cov = measurement_jacobian * covariance_in * measurement_jacobian.' + measurement_cov;
            kalman_gain = (covariance_in * measurement_jacobian.') / innovation_cov;

            updated_state = state_estimate + kalman_gain * measurement_residual;
            updated_covariance = (eye(3) - kalman_gain * measurement_jacobian) * covariance_in * (eye(3) - kalman_gain * measurement_jacobian).' + ...
                                 kalman_gain * measurement_cov * kalman_gain.'; % Joseph

            pose_out = Pose2(updated_state(1), updated_state(2), wrap_to_pi(updated_state(3)), ...
                "timestamp", pose_in.timestamp, ...
                "covariance", updated_covariance);

            covariance_out = updated_covariance;
        end

        function [pose_out, covariance_out] = meas_update_theta(obj, pose_in, covariance_in, heading_measurement, heading_variance)
            measurement_jacobian = [0 0 1];
            measurement_cov = heading_variance;

            state_estimate = [pose_in.x; pose_in.y; pose_in.theta];
            angle_residual = wrap_to_pi(heading_measurement - state_estimate(3));

            innovation_cov = measurement_jacobian * covariance_in * measurement_jacobian.' + measurement_cov;
            kalman_gain = (covariance_in * measurement_jacobian.') / innovation_cov;

            updated_state = state_estimate + kalman_gain * angle_residual;
            updated_state(3) = wrap_to_pi(updated_state(3));

            updated_covariance = (eye(3) - kalman_gain * measurement_jacobian) * covariance_in * (eye(3) - kalman_gain * measurement_jacobian).' + ...
                                 kalman_gain * measurement_cov * kalman_gain.'; % Joseph

            pose_out = Pose2(updated_state(1), updated_state(2), updated_state(3), ...
                "timestamp", pose_in.timestamp, ...
                "covariance", updated_covariance);

            covariance_out = updated_covariance;
        end

        function timestamp_out = time_from_start(~, start_time, time_sec)
            if isnat(start_time)
                timestamp_out = NaT;
            else
                timestamp_out = start_time + seconds(time_sec);
            end
        end
    end
end
