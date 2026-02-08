classdef LinearSigmaVarianceModel < RadarVarianceModel
    %LinearSigmaVarianceModel Linear sigma in range for both channels.

    methods
        function num_params = parameterCount(~)
            num_params = 4;
        end

        function [range_variance_m2, bearing_variance_rad2] = evaluate(~, range_m, theta, options)
            theta = theta(:);
            range_sigma_m = theta(1) + theta(2) .* range_m;
            bearing_sigma_rad = theta(3) + theta(4) .* range_m;

            range_sigma_m = max(range_sigma_m, options.sigma_floor_range_m);
            bearing_sigma_rad = max(bearing_sigma_rad, options.sigma_floor_bearing_rad);

            range_variance_m2 = range_sigma_m .^ 2;
            bearing_variance_rad2 = bearing_sigma_rad .^ 2;
        end

        function [range_jacobian, bearing_jacobian] = jacobian(~, range_m, theta, options)
            theta = theta(:);
            raw_range_sigma = theta(1) + theta(2) .* range_m;
            raw_bearing_sigma = theta(3) + theta(4) .* range_m;

            range_sigma = max(raw_range_sigma, options.sigma_floor_range_m);
            bearing_sigma = max(raw_bearing_sigma, options.sigma_floor_bearing_rad);

            range_jacobian = zeros(numel(range_m), 4);
            bearing_jacobian = zeros(numel(range_m), 4);

            is_range_free = raw_range_sigma > options.sigma_floor_range_m;
            is_bearing_free = raw_bearing_sigma > options.sigma_floor_bearing_rad;

            range_jacobian(is_range_free, 1) = 2 .* range_sigma(is_range_free);
            range_jacobian(is_range_free, 2) = 2 .* range_sigma(is_range_free) .* range_m(is_range_free);

            bearing_jacobian(is_bearing_free, 3) = 2 .* bearing_sigma(is_bearing_free);
            bearing_jacobian(is_bearing_free, 4) = 2 .* bearing_sigma(is_bearing_free) .* range_m(is_bearing_free);
        end

        function theta_initial = initialGuess(~, range_m, range_residual_m, bearing_residual_rad, options)
            num_bins = max(5, options.num_sigma_bins);
            bin_edges = approx_quantile_edges(range_m, num_bins);
            bin_edges(1) = -Inf;
            bin_edges(end) = Inf;

            bin_index = discretize(range_m, bin_edges);
            bin_center = zeros(num_bins, 1);
            bin_range_sigma = zeros(num_bins, 1);
            bin_bearing_sigma = zeros(num_bins, 1);
            valid_bin = false(num_bins, 1);

            for bin_id = 1:num_bins
                in_bin = (bin_index == bin_id);
                if nnz(in_bin) < 5
                    continue;
                end

                valid_bin(bin_id) = true;
                bin_center(bin_id) = mean(range_m(in_bin));

                range_sigma = sqrt(max(var(range_residual_m(in_bin), 0), 1e-12));
                bearing_sigma = sqrt(max(var(bearing_residual_rad(in_bin), 0), 1e-12));

                bin_range_sigma(bin_id) = max(range_sigma, options.sigma_floor_range_m);
                bin_bearing_sigma(bin_id) = max(bearing_sigma, options.sigma_floor_bearing_rad);
            end

            if nnz(valid_bin) < 2
                range_sigma_fallback = sqrt(max(var(range_residual_m, 0), 1e-12));
                bearing_sigma_fallback = sqrt(max(var(bearing_residual_rad, 0), 1e-12));

                sigma_range = [max(range_sigma_fallback, options.sigma_floor_range_m); 0.0];
                sigma_bearing = [max(bearing_sigma_fallback, options.sigma_floor_bearing_rad); 0.0];
            else
                regression_matrix = [ones(nnz(valid_bin), 1), bin_center(valid_bin)];
                sigma_range = regression_matrix \ bin_range_sigma(valid_bin);
                sigma_bearing = regression_matrix \ bin_bearing_sigma(valid_bin);
            end

            sigma_range = nudge_line_to_floor(sigma_range, range_m, options.sigma_floor_range_m);
            sigma_bearing = nudge_line_to_floor(sigma_bearing, range_m, options.sigma_floor_bearing_rad);

            theta_initial = [sigma_range(1); sigma_range(2); sigma_bearing(1); sigma_bearing(2)];
        end

        function param_labels = parameterLabels(~)
            param_labels = [
                "sigma_range_offset_m"
                "sigma_range_slope_m_per_m"
                "sigma_bearing_offset_rad"
                "sigma_bearing_slope_rad_per_m"
            ];
        end

        function [A, b] = inequalityConstraints(~, range_m, options)
            r_min = min(range_m);
            r_max = max(range_m);

            A = zeros(4, 4);
            b = zeros(4, 1);

            A(1,1) = -1; A(1,2) = -r_min; b(1) = -options.sigma_floor_range_m;
            A(2,1) = -1; A(2,2) = -r_max; b(2) = -options.sigma_floor_range_m;

            A(3,3) = -1; A(3,4) = -r_min; b(3) = -options.sigma_floor_bearing_rad;
            A(4,3) = -1; A(4,4) = -r_max; b(4) = -options.sigma_floor_bearing_rad;
        end
    end
end
