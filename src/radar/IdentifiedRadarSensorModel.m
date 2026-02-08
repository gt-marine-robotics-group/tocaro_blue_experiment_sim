classdef IdentifiedRadarSensorModel < RadarSensorModel
    % IdentifiedRadarSensorModel
    %
    % A RadarSensorModel implementation that matches the parameterization
    % used by the UPDATED SensorModelSolver:
    %
    %   bias is linear in predicted range:
    %       b_r(r) = beta_r0 + beta_r1 * r
    %       b_b(r) = beta_b0 + beta_b1 * r
    %
    %   sigma is linear in predicted range:
    %       sigma_r(r) = sigma_r0 + sigma_r1 * r
    %       sigma_b(r) = sigma_b0 + sigma_b1 * r
    %
    %   variance is sigma^2 (with optional small floors).
    %
    % This is convenient for plugging the fitted parameters back into the
    % existing simulation / evaluation code.

    properties
        enable_noise (1,1) logical = false

        % ---- Bias parameters ----
        beta_range_offset_m          (1,1) double {mustBeNumeric} = 0.0
        beta_range_slope_m_per_m     (1,1) double {mustBeNumeric} = 0.0
        beta_bearing_offset_rad      (1,1) double {mustBeNumeric} = 0.0
        beta_bearing_slope_rad_per_m (1,1) double {mustBeNumeric} = 0.0

        % ---- Sigma parameters (linear in range) ----
        sigma_range_offset_m         (1,1) double {mustBeNumeric} = 0.5
        sigma_range_slope_m_per_m    (1,1) double {mustBeNumeric} = 0.0
        sigma_bearing_offset_rad     (1,1) double {mustBeNumeric} = deg2rad(1.0)
        sigma_bearing_slope_rad_per_m(1,1) double {mustBeNumeric} = 0.0

        % ---- Numeric stability / clamps ----
        sigma_floor_range_m          (1,1) double {mustBeNumeric, mustBePositive} = 1e-4
        sigma_floor_bearing_rad      (1,1) double {mustBeNumeric, mustBePositive} = 1e-6

        % (Optional) tiny variance floors, mainly for log() safety in external code
        variance_floor_range_m2      (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.0
        variance_floor_bearing_rad2  (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.0

        % ---- Range clamp (optional) ----
        range_min_m (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.0
        range_max_m (1,1) double {mustBeNumeric, mustBePositive} = 1e6
    end

    methods
        function obj = IdentifiedRadarSensorModel(theta_vector, varargin)
            % Constructor
            %   IdentifiedRadarSensorModel(theta_vector, Name,Value,...)
            %
            % theta_vector can be [] (use defaults) or 8x1:
            %   [beta_r0 beta_r1 beta_b0 beta_b1 sigma_r0 sigma_r1 sigma_b0 sigma_b1]'

            if nargin >= 1 && ~isempty(theta_vector)
                theta_vector = theta_vector(:);
                if numel(theta_vector) ~= 8
                    error("IdentifiedRadarSensorModel:ThetaSize", "theta_vector must be 8x1.");
                end

                obj.beta_range_offset_m          = theta_vector(1);
                obj.beta_range_slope_m_per_m     = theta_vector(2);
                obj.beta_bearing_offset_rad      = theta_vector(3);
                obj.beta_bearing_slope_rad_per_m = theta_vector(4);

                obj.sigma_range_offset_m          = theta_vector(5);
                obj.sigma_range_slope_m_per_m     = theta_vector(6);
                obj.sigma_bearing_offset_rad      = theta_vector(7);
                obj.sigma_bearing_slope_rad_per_m = theta_vector(8);
            end

            % Name-value overrides
            if ~isempty(varargin)
                if mod(numel(varargin), 2) ~= 0
                    error("IdentifiedRadarSensorModel:ConstructorArgs", "Constructor expects name-value pairs.");
                end
                for arg_index = 1:2:numel(varargin)
                    property_name = varargin{arg_index};
                    property_value = varargin{arg_index + 1};
                    if ~isprop(obj, property_name)
                        error("IdentifiedRadarSensorModel:UnknownProperty", ...
                            "Unknown property: %s", string(property_name));
                    end
                    obj.(property_name) = property_value;
                end
            end
        end

        function measurements = measure(obj, target_poses)
            % measure Generate RadarMeasurement predictions for target_poses.
            %
            % Inputs:
            %   target_poses: Nx1 Pose2, already expressed in the RADAR frame.
            %
            % Outputs:
            %   measurements: Nx1 RadarMeasurement, each with:
            %       - theta, range (biased, and optionally noisy)
            %       - theta_bias, range_bias
            %       - theta_sigma, range_sigma

            if isempty(target_poses)
                measurements = RadarMeasurement.empty;
                return;
            end

            if ~isa(target_poses, "Pose2")
                error("IdentifiedRadarSensorModel:BadInput", "target_poses must be a Pose2 vector.");
            end

            num_targets = numel(target_poses);
            measurements(num_targets, 1) = RadarMeasurement();

            x_positions_m = reshape([target_poses.x], [], 1);
            y_positions_m = reshape([target_poses.y], [], 1);

            predicted_ranges_m = hypot(x_positions_m, y_positions_m);
            predicted_ranges_m = min(max(predicted_ranges_m, obj.range_min_m), obj.range_max_m);

            predicted_bearings_rad = atan2(y_positions_m, x_positions_m);

            % ---- Biases (linear) ----
            range_biases_m = obj.beta_range_offset_m + obj.beta_range_slope_m_per_m .* predicted_ranges_m;
            bearing_biases_rad = obj.beta_bearing_offset_rad + obj.beta_bearing_slope_rad_per_m .* predicted_ranges_m;

            % ---- Sigmas (linear; clamped to floors) ----
            range_sigmas_m = obj.sigma_range_offset_m + obj.sigma_range_slope_m_per_m .* predicted_ranges_m;
            bearing_sigmas_rad = obj.sigma_bearing_offset_rad + obj.sigma_bearing_slope_rad_per_m .* predicted_ranges_m;

            range_sigmas_m = max(range_sigmas_m, obj.sigma_floor_range_m);
            bearing_sigmas_rad = max(bearing_sigmas_rad, obj.sigma_floor_bearing_rad);

            % Optional variance floors (rarely needed; kept for compatibility)
            range_variance_m2 = range_sigmas_m.^2 + obj.variance_floor_range_m2;
            bearing_variance_rad2 = bearing_sigmas_rad.^2 + obj.variance_floor_bearing_rad2;

            range_sigmas_m = sqrt(range_variance_m2);
            bearing_sigmas_rad = sqrt(bearing_variance_rad2);

            % ---- Noise ----
            if obj.enable_noise
                range_noise_m = range_sigmas_m .* randn(num_targets, 1);
                bearing_noise_rad = bearing_sigmas_rad .* randn(num_targets, 1);
            else
                range_noise_m = zeros(num_targets, 1);
                bearing_noise_rad = zeros(num_targets, 1);
            end

            measured_ranges_m = predicted_ranges_m + range_biases_m + range_noise_m;
            measured_ranges_m = max(measured_ranges_m, 1e-6); % keep strictly positive

            measured_bearings_rad = predicted_bearings_rad + bearing_biases_rad + bearing_noise_rad;
            measured_bearings_rad = wrap_to_pi(measured_bearings_rad);

            for target_index = 1:num_targets
                meas = RadarMeasurement();

                meas.theta = measured_bearings_rad(target_index);
                meas.range = measured_ranges_m(target_index);

                meas.theta_sigma = bearing_sigmas_rad(target_index);
                meas.range_sigma = range_sigmas_m(target_index);

                meas.theta_bias = bearing_biases_rad(target_index);
                meas.range_bias = range_biases_m(target_index);

                meas.target_id = "";
                measurements(target_index) = meas;
            end
        end
    end

    methods (Static, Access = private)
        function wrapped_angle = wrapToPi(angle_rad)
            wrapped_angle = wrap_to_pi(angle_rad);
        end
    end
end





% classdef IdentifiedRadarSensorModel < RadarSensorModel
%     % IdentifiedRadarSensorModel
%     %
%     % A RadarSensorModel implementation that matches the parameterization
%     % used by SensorModelSolver (linear bias in range_hat, log-linear
%     % variance in range_hat).
%     %
%     % This is convenient for plugging the fitted parameters back into the
%     % existing simulation / evaluation code.

%     properties
%         enable_noise (1,1) logical = false

%         % ---- Bias parameters ----
%         beta_range_offset_m        (1,1) double {mustBeNumeric} = 0.0
%         beta_range_slope_m_per_m   (1,1) double {mustBeNumeric} = 0.0
%         beta_bearing_offset_rad      (1,1) double {mustBeNumeric} = 0.0
%         beta_bearing_slope_rad_per_m (1,1) double {mustBeNumeric} = 0.0

%         % ---- Log-variance parameters ----
%         gamma_range_offset_logvar        (1,1) double {mustBeNumeric} = log(0.5^2)
%         gamma_range_slope_logvar_per_m   (1,1) double {mustBeNumeric} = 0.0
%         gamma_bearing_offset_logvar      (1,1) double {mustBeNumeric} = log(deg2rad(1.0)^2)
%         gamma_bearing_slope_logvar_per_m (1,1) double {mustBeNumeric} = 0.0

%         % ---- Numeric stability floors ----
%         variance_floor_range_m2    (1,1) double {mustBeNumeric, mustBeNonnegative} = 1e-12
%         variance_floor_bearing_rad2 (1,1) double {mustBeNumeric, mustBeNonnegative} = 1e-12

%         % ---- Range clamp (optional) ----
%         range_min_m (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.0
%         range_max_m (1,1) double {mustBeNumeric, mustBePositive} = 1e6
%     end

%     methods
%         function obj = IdentifiedRadarSensorModel(theta_vector, varargin)
%             % Constructor
%             %   IdentifiedRadarSensorModel(theta_vector, Name,Value,...)
%             %
%             % theta_vector can be [] (use defaults) or 8x1:
%             %   [beta_r0 beta_r1 beta_b0 beta_b1 gamma_r0 gamma_r1 gamma_b0 gamma_b1]'

%             if nargin >= 1 && ~isempty(theta_vector)
%                 theta_vector = theta_vector(:);
%                 if numel(theta_vector) ~= 8
%                     error("IdentifiedRadarSensorModel:ThetaSize", "theta_vector must be 8x1.");
%                 end
%                 obj.beta_range_offset_m = theta_vector(1);
%                 obj.beta_range_slope_m_per_m = theta_vector(2);
%                 obj.beta_bearing_offset_rad = theta_vector(3);
%                 obj.beta_bearing_slope_rad_per_m = theta_vector(4);

%                 obj.gamma_range_offset_logvar = theta_vector(5);
%                 obj.gamma_range_slope_logvar_per_m = theta_vector(6);
%                 obj.gamma_bearing_offset_logvar = theta_vector(7);
%                 obj.gamma_bearing_slope_logvar_per_m = theta_vector(8);
%             end

%             % Name-value overrides
%             if ~isempty(varargin)
%                 if mod(numel(varargin), 2) ~= 0
%                     error("IdentifiedRadarSensorModel:ConstructorArgs", "Constructor expects name-value pairs.");
%                 end
%                 for arg_index = 1:2:numel(varargin)
%                     property_name = varargin{arg_index};
%                     property_value = varargin{arg_index + 1};
%                     if ~isprop(obj, property_name)
%                         error("IdentifiedRadarSensorModel:UnknownProperty", "Unknown property: %s", string(property_name));
%                     end
%                     obj.(property_name) = property_value;
%                 end
%             end
%         end

%         function measurements = measure(obj, target_poses)
%             % measure Generate RadarMeasurement predictions for target_poses.
%             %
%             % Inputs:
%             %   target_poses: Nx1 Pose2, already expressed in the RADAR frame.
%             %
%             % Outputs:
%             %   measurements: Nx1 RadarMeasurement, each with:
%             %       - theta, range (biased, and optionally noisy)
%             %       - theta_bias, range_bias
%             %       - theta_sigma, range_sigma

%             if isempty(target_poses)
%                 measurements = RadarMeasurement.empty;
%                 return;
%             end

%             if ~isa(target_poses, "Pose2")
%                 error("IdentifiedRadarSensorModel:BadInput", "target_poses must be a Pose2 vector.");
%             end

%             num_targets = numel(target_poses);
%             measurements(num_targets, 1) = RadarMeasurement();

%             x_positions_m = reshape([target_poses.x], [], 1);
%             y_positions_m = reshape([target_poses.y], [], 1);

%             predicted_ranges_m = hypot(x_positions_m, y_positions_m);
%             predicted_ranges_m = min(max(predicted_ranges_m, obj.range_min_m), obj.range_max_m);
%             predicted_bearings_rad = atan2(y_positions_m, x_positions_m);

%             range_biases_m = obj.beta_range_offset_m + obj.beta_range_slope_m_per_m .* predicted_ranges_m;
%             bearing_biases_rad = obj.beta_bearing_offset_rad + obj.beta_bearing_slope_rad_per_m .* predicted_ranges_m;

%             range_variance_m2 = exp(obj.gamma_range_offset_logvar + obj.gamma_range_slope_logvar_per_m .* predicted_ranges_m) + obj.variance_floor_range_m2;
%             bearing_variance_rad2 = exp(obj.gamma_bearing_offset_logvar + obj.gamma_bearing_slope_logvar_per_m .* predicted_ranges_m) + obj.variance_floor_bearing_rad2;

%             range_sigmas_m = sqrt(range_variance_m2);
%             bearing_sigmas_rad = sqrt(bearing_variance_rad2);

%             if obj.enable_noise
%                 range_noise_m = range_sigmas_m .* randn(num_targets, 1);
%                 bearing_noise_rad = bearing_sigmas_rad .* randn(num_targets, 1);
%             else
%                 range_noise_m = zeros(num_targets, 1);
%                 bearing_noise_rad = zeros(num_targets, 1);
%             end

%             measured_ranges_m = predicted_ranges_m + range_biases_m + range_noise_m;
%             measured_ranges_m = max(measured_ranges_m, 1e-6); % keep strictly positive

%             measured_bearings_rad = predicted_bearings_rad + bearing_biases_rad + bearing_noise_rad;
%             measured_bearings_rad = IdentifiedRadarSensorModel.wrapToPi(measured_bearings_rad);

%             for target_index = 1:num_targets
%                 meas = RadarMeasurement();
%                 meas.theta = measured_bearings_rad(target_index);
%                 meas.range = measured_ranges_m(target_index);

%                 meas.theta_sigma = bearing_sigmas_rad(target_index);
%                 meas.range_sigma = range_sigmas_m(target_index);

%                 meas.theta_bias = bearing_biases_rad(target_index);
%                 meas.range_bias = range_biases_m(target_index);

%                 meas.target_id = "";
%                 measurements(target_index) = meas;
%             end
%         end
%     end

%     methods (Static, Access = private)
%         function wrapped_angle = wrapToPi(angle_rad)
%             wrapped_angle = atan2(sin(angle_rad), cos(angle_rad));
%         end
%     end
% end
