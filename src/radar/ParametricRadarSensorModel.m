classdef ParametricRadarSensorModel < RadarSensorModel
    %ParametricRadarSensorModel Range-dependent bias + variance models.

    properties

        bias_model RadarBiasModel = LinearBiasModel()
        variance_model RadarVarianceModel = LinearSigmaVarianceModel()

        bias_parameters (:,1) double = zeros(0, 1)
        variance_parameters (:,1) double = zeros(0, 1)

        enable_noise (1,1) logical = true

        variance_options struct = struct( ...
            "sigma_floor_range_m", 1e-6, ...
            "sigma_floor_bearing_rad", 1e-8, ...
            "variance_floor_range_m2", 0.0, ...
            "variance_floor_bearing_rad2", 0.0, ...
            "num_sigma_bins", 10)

        range_min_m (1,1) double {mustBeNumeric, mustBeNonnegative} = 0.0
        range_max_m (1,1) double {mustBeNumeric, mustBePositive} = 1e6
    end

    methods
        function obj = ParametricRadarSensorModel(bias_model, variance_model, varargin)
            if nargin >= 1
                obj.bias_model = bias_model;
            end
            if nargin >= 2
                obj.variance_model = variance_model;
            end

            if ~isempty(varargin)
                if mod(numel(varargin), 2) ~= 0
                    error("ParametricRadarSensorModel:ConstructorArgs", "Constructor expects name-value pairs.");
                end
                for arg_index = 1:2:numel(varargin)
                    property_name = varargin{arg_index};
                    property_value = varargin{arg_index + 1};
                    if ~isprop(obj, property_name)
                        error("ParametricRadarSensorModel:UnknownProperty", ...
                            "Unknown property: %s", string(property_name));
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

            if ~isa(target_poses, "Pose2")
                error("ParametricRadarSensorModel:BadInput", "target_poses must be a Pose2 vector.");
            end

            if isempty(obj.bias_model) || isempty(obj.variance_model)
                error("ParametricRadarSensorModel:MissingModels", "bias_model and variance_model must be set.");
            end
            if isempty(obj.bias_parameters) || isempty(obj.variance_parameters)
                error("ParametricRadarSensorModel:MissingParameters", "bias_parameters and variance_parameters must be set.");
            end

            if numel(obj.bias_parameters) ~= obj.bias_model.parameterCount()
                error("ParametricRadarSensorModel:BiasParamCount", ...
                    "bias_parameters must have %d entries.", obj.bias_model.parameterCount());
            end
            if numel(obj.variance_parameters) ~= obj.variance_model.parameterCount()
                error("ParametricRadarSensorModel:VarianceParamCount", ...
                    "variance_parameters must have %d entries.", obj.variance_model.parameterCount());
            end

            num_targets = numel(target_poses);
            measurements(num_targets, 1) = RadarMeasurement();

            x_positions_m = reshape([target_poses.x], [], 1);
            y_positions_m = reshape([target_poses.y], [], 1);

            predicted_ranges_m = hypot(x_positions_m, y_positions_m);
            predicted_ranges_m = min(max(predicted_ranges_m, obj.range_min_m), obj.range_max_m);

            predicted_bearings_rad = atan2(y_positions_m, x_positions_m);

            [range_biases_m, bearing_biases_rad] = obj.bias_model.evaluate(predicted_ranges_m, obj.bias_parameters);

            [range_variance_m2, bearing_variance_rad2] = obj.variance_model.evaluate( ...
                predicted_ranges_m, obj.variance_parameters, obj.variance_options);

            range_sigmas_m = sqrt(range_variance_m2);
            bearing_sigmas_rad = sqrt(bearing_variance_rad2);

            if obj.enable_noise
                range_noise_m = range_sigmas_m .* randn(num_targets, 1);
                bearing_noise_rad = bearing_sigmas_rad .* randn(num_targets, 1);
            else
                range_noise_m = zeros(num_targets, 1);
                bearing_noise_rad = zeros(num_targets, 1);
            end

            measured_ranges_m = predicted_ranges_m + range_biases_m + range_noise_m;
            measured_ranges_m = max(measured_ranges_m, 1e-6);

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
end
