classdef SimpleSensorModel < RadarSensorModel
    % SimpleSensorModel
    % Range-dependent bias and uncertainty in bearing (theta) and range.
    %
    % Inputs:
    %   target_poses: Nx1 Pose2, already expressed in the RADAR frame.
    %
    % Outputs:
    %   measurements: Nx1 RadarMeasurement, each with:
    %       - theta, range (biased + noisy)
    %       - theta_bias, range_bias (the applied bias values)
    %       - theta_sigma, range_sigma (the applied sigma values)

    properties
        enable_noise (1,1) logical = true

        % ---- Range-dependent BIAS models ----
        % bias_range(r) = bias_range_offset + bias_range_slope * r + bias_range_quad * r^2
        range_bias_offset_m     (1,1) double {mustBeNumeric} = 0.0
        range_bias_slope_m_per_m (1,1) double {mustBeNumeric} = 0.0
        range_bias_quad_m_per_m2 (1,1) double {mustBeNumeric} = 0.0

        % bias_theta(r) = bias_theta_offset + bias_theta_slope * r + bias_theta_quad * r^2
        theta_bias_offset_rad      (1,1) double {mustBeNumeric} = 0.0
        theta_bias_slope_rad_per_m (1,1) double {mustBeNumeric} = 0.0
        theta_bias_quad_rad_per_m2 (1,1) double {mustBeNumeric} = 0.0

        % ---- Range-dependent SIGMA (uncertainty) models ----
        % sigma_range(r) = max(range_sigma_min, range_sigma_offset + range_sigma_slope * r)
        range_sigma_min_m      (1,1) double {mustBeNumeric, mustBePositive} = 1e-3
        range_sigma_offset_m   (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.05
        range_sigma_slope_m_per_m (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.001

        % sigma_theta(r) = max(theta_sigma_min, theta_sigma_offset + theta_sigma_slope * r)
        theta_sigma_min_rad        (1,1) double {mustBeNumeric, mustBePositive} = 1e-6
        theta_sigma_offset_rad     (1,1) double {mustBeNumeric, mustBeNonnegative} = deg2rad(0.2)
        theta_sigma_slope_rad_per_m (1,1) double {mustBeNumeric, mustBeNonnegative} = deg2rad(0.01)

        % ---- Range clamp ----
        range_min_m (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.0
        range_max_m (1,1) double {mustBeNumeric, mustBePositive} = 1e6
    end

    methods
        function obj = SimpleSensorModel(varargin)
            % Name-value constructor
            if ~isempty(varargin)
                if mod(numel(varargin), 2) ~= 0
                    error("SimpleSensorModel:ConstructorArgs", "Constructor expects name-value pairs.");
                end
                for arg_index = 1:2:numel(varargin)
                    property_name = varargin{arg_index};
                    property_value = varargin{arg_index + 1};
                    if ~isprop(obj, property_name)
                        error("SimpleSensorModel:UnknownProperty", "Unknown property: %s", string(property_name));
                    end
                    obj.(property_name) = property_value;
                end
            end
        end

        function measurements = measure(obj, target_poses)
            if isempty(target_poses)
                measurements = RadarMeasurement.empty;
                return;
            end

            num_targets = numel(target_poses);
            measurements(num_targets, 1) = RadarMeasurement();

            % --- True polar quantities from Pose2 (already in radar frame) ---
            x_positions = reshape([target_poses.x], [], 1);
            y_positions = reshape([target_poses.y], [], 1);

            true_ranges = hypot(x_positions, y_positions);
            true_ranges = min(max(true_ranges, obj.range_min_m), obj.range_max_m);

            true_thetas = atan2(y_positions, x_positions);

            % --- Range-dependent bias and sigma ---
            range_biases = obj.computeRangeBias(true_ranges);
            theta_biases = obj.computeThetaBias(true_ranges);

            range_sigmas = obj.computeRangeSigma(true_ranges);
            theta_sigmas = obj.computeThetaSigma(true_ranges);

            % --- Noise sampling ---
            if obj.enable_noise
                range_noise = range_sigmas .* randn(num_targets, 1);
                theta_noise = theta_sigmas .* randn(num_targets, 1);
            else
                range_noise = zeros(num_targets, 1);
                theta_noise = zeros(num_targets, 1);
            end

            % --- Apply bias + noise ---
            measured_ranges = true_ranges + range_biases + range_noise;
            measured_ranges = max(measured_ranges, obj.range_sigma_min_m); % keep strictly positive

            measured_thetas = true_thetas + theta_biases + theta_noise;
            measured_thetas = atan2(sin(measured_thetas), cos(measured_thetas)); % wrap [-pi, pi]

            % --- Pack outputs ---
            for target_index = 1:num_targets
                meas = RadarMeasurement();
                meas.theta = measured_thetas(target_index);
                meas.range = measured_ranges(target_index);

                meas.theta_sigma = theta_sigmas(target_index);
                meas.range_sigma = range_sigmas(target_index);

                meas.theta_bias = theta_biases(target_index);
                meas.range_bias = range_biases(target_index);

                meas.target_id = ""; % optional: set externally if you have IDs
                measurements(target_index) = meas;
            end
        end
    end

    methods (Access = private)
        function range_biases = computeRangeBias(obj, ranges_m)
            range_biases = obj.range_bias_offset_m ...
                         + obj.range_bias_slope_m_per_m .* ranges_m ...
                         + obj.range_bias_quad_m_per_m2 .* (ranges_m .^ 2);
        end

        function theta_biases = computeThetaBias(obj, ranges_m)
            theta_biases = obj.theta_bias_offset_rad ...
                         + obj.theta_bias_slope_rad_per_m .* ranges_m ...
                         + obj.theta_bias_quad_rad_per_m2 .* (ranges_m .^ 2);
        end

        function range_sigmas = computeRangeSigma(obj, ranges_m)
            range_sigmas = obj.range_sigma_offset_m + obj.range_sigma_slope_m_per_m .* ranges_m;
            range_sigmas = max(range_sigmas, obj.range_sigma_min_m);
        end

        function theta_sigmas = computeThetaSigma(obj, ranges_m)
            theta_sigmas = obj.theta_sigma_offset_rad + obj.theta_sigma_slope_rad_per_m .* ranges_m;
            theta_sigmas = max(theta_sigmas, obj.theta_sigma_min_rad);
        end
    end
end
