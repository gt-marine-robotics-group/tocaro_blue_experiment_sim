classdef LinearSigmaVarianceFunction < RangeVarianceFunction
    %LinearSigmaVarianceFunction Linear sigma model for a single channel.

    methods
        function num_params = parameterCount(~)
            num_params = 2;
        end

        function variance = evaluate(~, range_m, theta, options)
            theta = theta(:);
            sigma = theta(1) + theta(2) .* range_m;
            sigma = max(sigma, options.sigma_floor);
            variance = sigma .^ 2;
        end

        function jacobian = jacobian(~, range_m, theta, options)
            theta = theta(:);
            raw_sigma = theta(1) + theta(2) .* range_m;
            sigma = max(raw_sigma, options.sigma_floor);

            jacobian = zeros(numel(range_m), 2);
            is_free = raw_sigma > options.sigma_floor;

            jacobian(is_free, 1) = 2 .* sigma(is_free);
            jacobian(is_free, 2) = 2 .* sigma(is_free) .* range_m(is_free);
        end

        function theta_initial = initialGuess(~, range_m, residual, options)
            num_bins = max(5, options.num_sigma_bins);
            bin_edges = approx_quantile_edges(range_m, num_bins);
            bin_edges(1) = -Inf;
            bin_edges(end) = Inf;

            bin_index = discretize(range_m, bin_edges);
            bin_center = zeros(num_bins, 1);
            bin_sigma = zeros(num_bins, 1);
            valid_bin = false(num_bins, 1);

            for bin_id = 1:num_bins
                in_bin = (bin_index == bin_id);
                if nnz(in_bin) < 5
                    continue;
                end

                valid_bin(bin_id) = true;
                bin_center(bin_id) = mean(range_m(in_bin));

                bin_sigma(bin_id) = max(sqrt(max(var(residual(in_bin), 0), 1e-12)), options.sigma_floor);
            end

            if nnz(valid_bin) < 2
                sigma_fallback = max(sqrt(max(var(residual, 0), 1e-12)), options.sigma_floor);
                sigma_coeff = [sigma_fallback; 0.0];
            else
                regression_matrix = [ones(nnz(valid_bin), 1), bin_center(valid_bin)];
                sigma_coeff = regression_matrix \ bin_sigma(valid_bin);
            end

            sigma_coeff = nudge_line_to_floor(sigma_coeff, range_m, options.sigma_floor);
            theta_initial = [sigma_coeff(1); sigma_coeff(2)];
        end

        function param_labels = parameterLabels(~, unit_label)
            param_labels = [
                "sigma_offset_" + unit_label
                "sigma_slope_" + unit_label + "_per_m"
            ];
        end

        function [A, b] = inequalityConstraints(~, range_m, options)
            r_min = min(range_m);
            r_max = max(range_m);

            A = zeros(2, 2);
            b = zeros(2, 1);

            A(1, 1) = -1; A(1, 2) = -r_min; b(1) = -options.sigma_floor;
            A(2, 1) = -1; A(2, 2) = -r_max; b(2) = -options.sigma_floor;
        end
    end
end
