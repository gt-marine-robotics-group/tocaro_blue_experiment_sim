classdef QuadraticBiasModel < RadarBiasModel
    %QuadraticBiasModel Quadratic bias in predicted range for range and bearing.

    methods
        function num_params = parameterCount(~)
            num_params = 6;
        end

        function [range_bias_m, bearing_bias_rad] = evaluate(~, range_m, theta)
            theta = theta(:);
            range_bias_m = theta(1) + theta(2) .* range_m + theta(3) .* (range_m .^ 2);
            bearing_bias_rad = theta(4) + theta(5) .* range_m + theta(6) .* (range_m .^ 2);
        end

        function [range_jacobian, bearing_jacobian] = jacobian(~, range_m, ~)
            num_samples = numel(range_m);
            range_jacobian = zeros(num_samples, 6);
            bearing_jacobian = zeros(num_samples, 6);

            range_jacobian(:, 1) = 1;
            range_jacobian(:, 2) = range_m;
            range_jacobian(:, 3) = range_m .^ 2;

            bearing_jacobian(:, 4) = 1;
            bearing_jacobian(:, 5) = range_m;
            bearing_jacobian(:, 6) = range_m .^ 2;
        end

        function theta_initial = initialGuess(~, range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad)
            num_samples = numel(range_m);
            design_matrix = [ones(num_samples, 1), range_m(:), range_m(:).^2];

            range_residual_raw_m = measured_range_m(:) - range_m(:);
            bearing_residual_raw_rad = wrap_to_pi(measured_bearing_rad(:) - predicted_bearing_rad(:));

            beta_range = design_matrix \ range_residual_raw_m;
            beta_bearing = design_matrix \ bearing_residual_raw_rad;

            theta_initial = [beta_range(1); beta_range(2); beta_range(3); beta_bearing(1); beta_bearing(2); beta_bearing(3)];
        end

        function param_labels = parameterLabels(~)
            param_labels = [
                "beta_range_offset_m"
                "beta_range_slope_m_per_m"
                "beta_range_quad_m_per_m2"
                "beta_bearing_offset_rad"
                "beta_bearing_slope_rad_per_m"
                "beta_bearing_quad_rad_per_m2"
            ];
        end
    end
end
