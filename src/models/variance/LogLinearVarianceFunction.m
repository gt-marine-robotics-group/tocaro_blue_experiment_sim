classdef LogLinearVarianceFunction < RangeVarianceFunction
    %LogLinearVarianceFunction Log-linear variance model for a single channel.

    methods
        function num_params = parameterCount(~)
            num_params = 2;
        end

        function variance = evaluate(~, range_m, theta, options)
            theta = theta(:);
            log_variance = theta(1) + theta(2) .* range_m;
            variance = exp(log_variance) + options.variance_floor;
        end

        function jacobian = jacobian(~, range_m, theta, options)
            theta = theta(:);
            log_variance = theta(1) + theta(2) .* range_m;
            variance = exp(log_variance);
            jacobian = zeros(numel(range_m), 2);
            jacobian(:, 1) = variance;
            jacobian(:, 2) = variance .* range_m;
        end

        function theta_initial = initialGuess(~, range_m, residual, options)
            num_bins = max(5, options.num_sigma_bins);
            bin_edges = approx_quantile_edges(range_m, num_bins);
            bin_edges(1) = -Inf;
            bin_edges(end) = Inf;

            bin_index = discretize(range_m, bin_edges);
            bin_center = zeros(num_bins, 1);
            bin_var = zeros(num_bins, 1);
            valid_bin = false(num_bins, 1);

            for bin_id = 1:num_bins
                in_bin = (bin_index == bin_id);
                if nnz(in_bin) < 5
                    continue;
                end

                valid_bin(bin_id) = true;
                bin_center(bin_id) = mean(range_m(in_bin));
                bin_var(bin_id) = max(var(residual(in_bin), 0), 1e-12);
            end

            if nnz(valid_bin) < 2
                var_fallback = max(var(residual, 0), 1e-12);
                gamma = [log(var_fallback); 0.0];
            else
                regression_matrix = [ones(nnz(valid_bin), 1), bin_center(valid_bin)];
                gamma = regression_matrix \ log(bin_var(valid_bin));
            end

            theta_initial = [gamma(1); gamma(2)];
        end

        function param_labels = parameterLabels(~, ~)
            param_labels = [
                "logvar_offset"
                "logvar_slope_per_m"
            ];
        end

        function [A, b] = inequalityConstraints(~, ~, ~)
            A = [];
            b = [];
        end
    end
end
