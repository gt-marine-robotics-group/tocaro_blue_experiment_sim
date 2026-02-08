classdef MixedBiasModel < RadarBiasModel
    %MixedBiasModel Independent bias functions for range and bearing.

    properties
        range_model RangeBiasFunction = LinearBiasFunction()
        bearing_model RangeBiasFunction = LinearBiasFunction()
    end

    methods
        function obj = MixedBiasModel(range_model, bearing_model)
            if nargin >= 1
                obj.range_model = range_model;
            end
            if nargin >= 2
                obj.bearing_model = bearing_model;
            end
        end

        function num_params = parameterCount(obj)
            num_params = obj.range_model.parameterCount() + obj.bearing_model.parameterCount();
        end

        function [range_bias_m, bearing_bias_rad] = evaluate(obj, range_m, theta)
            theta = theta(:);
            range_count = obj.range_model.parameterCount();
            range_theta = theta(1:range_count);
            bearing_theta = theta(range_count + 1:end);

            range_bias_m = obj.range_model.evaluate(range_m, range_theta);
            bearing_bias_rad = obj.bearing_model.evaluate(range_m, bearing_theta);
        end

        function [range_jacobian, bearing_jacobian] = jacobian(obj, range_m, theta)
            theta = theta(:);
            range_count = obj.range_model.parameterCount();
            bearing_count = obj.bearing_model.parameterCount();
            range_theta = theta(1:range_count);
            bearing_theta = theta(range_count + 1:end);

            range_partial = obj.range_model.jacobian(range_m, range_theta);
            bearing_partial = obj.bearing_model.jacobian(range_m, bearing_theta);

            num_samples = numel(range_m);
            total_params = range_count + bearing_count;

            range_jacobian = zeros(num_samples, total_params);
            bearing_jacobian = zeros(num_samples, total_params);

            range_jacobian(:, 1:range_count) = range_partial;
            bearing_jacobian(:, range_count + 1:end) = bearing_partial;
        end

        function theta_initial = initialGuess(obj, range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad)
            range_residual_raw_m = measured_range_m(:) - range_m(:);
            bearing_residual_raw_rad = wrap_to_pi(measured_bearing_rad(:) - predicted_bearing_rad(:));

            range_theta = obj.range_model.initialGuess(range_m, range_residual_raw_m);
            bearing_theta = obj.bearing_model.initialGuess(range_m, bearing_residual_raw_rad);

            theta_initial = [range_theta(:); bearing_theta(:)];
        end

        function param_labels = parameterLabels(obj)
            range_labels = obj.range_model.parameterLabels("m");
            bearing_labels = obj.bearing_model.parameterLabels("rad");

            param_labels = [
                "beta_range_" + range_labels(:)
                "beta_bearing_" + bearing_labels(:)
            ];
        end
    end
end
