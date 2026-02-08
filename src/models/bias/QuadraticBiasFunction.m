classdef QuadraticBiasFunction < RangeBiasFunction
    %QuadraticBiasFunction Quadratic bias function for a single channel.

    methods
        function num_params = parameterCount(~)
            num_params = 3;
        end

        function bias = evaluate(~, range_m, theta)
            theta = theta(:);
            bias = theta(1) + theta(2) .* range_m + theta(3) .* (range_m .^ 2);
        end

        function jacobian = jacobian(~, range_m, ~)
            num_samples = numel(range_m);
            jacobian = zeros(num_samples, 3);
            jacobian(:, 1) = 1;
            jacobian(:, 2) = range_m;
            jacobian(:, 3) = range_m .^ 2;
        end

        function theta_initial = initialGuess(~, range_m, residual)
            num_samples = numel(range_m);
            design_matrix = [ones(num_samples, 1), range_m(:), range_m(:) .^ 2];
            beta = design_matrix \residual(:);
            theta_initial = [beta(1); beta(2); beta(3)];
        end

        function param_labels = parameterLabels(~, unit_label)
            param_labels = [
                "offset_" + unit_label
                "slope_" + unit_label + "_per_m"
                "quad_" + unit_label + "_per_m2"
            ];
        end
    end
end
