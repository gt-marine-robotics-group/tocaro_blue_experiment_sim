classdef LogLinearVarianceModel < RadarVarianceModel
    %LogLinearVarianceModel Log-linear variance in predicted range.

    methods
        function num_params = parameterCount(~)
            num_params = 4;
        end

        function [range_variance_m2, bearing_variance_rad2] = evaluate(~, range_m, theta, options)
            theta = theta(:);
            range_log_variance = theta(1) + theta(2) .* range_m;
            bearing_log_variance = theta(3) + theta(4) .* range_m;

            range_variance_m2 = exp(range_log_variance) + options.variance_floor_range_m2;
            bearing_variance_rad2 = exp(bearing_log_variance) + options.variance_floor_bearing_rad2;
        end

        function [range_jacobian, bearing_jacobian] = jacobian(~, range_m, theta, options)
            theta = theta(:);
            range_log_variance = theta(1) + theta(2) .* range_m;
            bearing_log_variance = theta(3) + theta(4) .* range_m;

            range_variance_m2 = exp(range_log_variance);
            bearing_variance_rad2 = exp(bearing_log_variance);

            range_jacobian = zeros(numel(range_m), 4);
            bearing_jacobian = zeros(numel(range_m), 4);

            range_jacobian(:, 1) = range_variance_m2;
            range_jacobian(:, 2) = range_variance_m2 .* range_m;

            bearing_jacobian(:, 3) = bearing_variance_rad2;
            bearing_jacobian(:, 4) = bearing_variance_rad2 .* range_m;
        end

        function theta_initial = initialGuess(~, range_m, range_residual_m, bearing_residual_rad, options)
            num_bins = max(5, options.num_sigma_bins);
            bin_edges = approx_quantile_edges(range_m, num_bins);
            bin_edges(1) = -Inf;
            bin_edges(end) = Inf;

            bin_index = discretize(range_m, bin_edges);
            bin_center = zeros(num_bins, 1);
            bin_range_var = zeros(num_bins, 1);
            bin_bearing_var = zeros(num_bins, 1);
            valid_bin = false(num_bins, 1);

            for bin_id = 1:num_bins
                in_bin = (bin_index == bin_id);
                if nnz(in_bin) < 5
                    continue;
                end

                valid_bin(bin_id) = true;
                bin_center(bin_id) = mean(range_m(in_bin));

                bin_range_var(bin_id) = max(var(range_residual_m(in_bin), 0), 1e-12);
                bin_bearing_var(bin_id) = max(var(bearing_residual_rad(in_bin), 0), 1e-12);
            end

            if nnz(valid_bin) < 2
                range_var_fallback = max(var(range_residual_m, 0), 1e-12);
                bearing_var_fallback = max(var(bearing_residual_rad, 0), 1e-12);

                gamma_range = [log(range_var_fallback); 0.0];
                gamma_bearing = [log(bearing_var_fallback); 0.0];
            else
                regression_matrix = [ones(nnz(valid_bin), 1), bin_center(valid_bin)];
                gamma_range = regression_matrix \ log(bin_range_var(valid_bin));
                gamma_bearing = regression_matrix \ log(bin_bearing_var(valid_bin));
            end

            theta_initial = [gamma_range(1); gamma_range(2); gamma_bearing(1); gamma_bearing(2)];
        end

        function param_labels = parameterLabels(~)
            param_labels = [
                "gamma_range_offset_logvar"
                "gamma_range_slope_logvar_per_m"
                "gamma_bearing_offset_logvar"
                "gamma_bearing_slope_logvar_per_m"
            ];
        end

        function [A, b] = inequalityConstraints(~, ~, ~)
            A = [];
            b = [];
        end
    end
end
