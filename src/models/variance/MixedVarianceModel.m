classdef MixedVarianceModel < RadarVarianceModel
    %MixedVarianceModel Independent variance functions for range and bearing.

    properties
        range_model RangeVarianceFunction = LinearSigmaVarianceFunction()
        bearing_model RangeVarianceFunction = LinearSigmaVarianceFunction()
    end

    methods
        function obj = MixedVarianceModel(range_model, bearing_model)
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

        function [range_variance_m2, bearing_variance_rad2] = evaluate(obj, range_m, theta, options)
            theta = theta(:);
            range_count = obj.range_model.parameterCount();
            range_theta = theta(1:range_count);
            bearing_theta = theta(range_count + 1:end);

            range_options = obj.channelOptions(options, true);
            bearing_options = obj.channelOptions(options, false);

            range_variance_m2 = obj.range_model.evaluate(range_m, range_theta, range_options);
            bearing_variance_rad2 = obj.bearing_model.evaluate(range_m, bearing_theta, bearing_options);
        end

        function [range_jacobian, bearing_jacobian] = jacobian(obj, range_m, theta, options)
            theta = theta(:);
            range_count = obj.range_model.parameterCount();
            bearing_count = obj.bearing_model.parameterCount();
            range_theta = theta(1:range_count);
            bearing_theta = theta(range_count + 1:end);

            range_options = obj.channelOptions(options, true);
            bearing_options = obj.channelOptions(options, false);

            range_partial = obj.range_model.jacobian(range_m, range_theta, range_options);
            bearing_partial = obj.bearing_model.jacobian(range_m, bearing_theta, bearing_options);

            num_samples = numel(range_m);
            total_params = range_count + bearing_count;

            range_jacobian = zeros(num_samples, total_params);
            bearing_jacobian = zeros(num_samples, total_params);

            range_jacobian(:, 1:range_count) = range_partial;
            bearing_jacobian(:, range_count + 1:end) = bearing_partial;
        end

        function theta_initial = initialGuess(obj, range_m, range_residual_m, bearing_residual_rad, options)
            range_options = obj.channelOptions(options, true);
            bearing_options = obj.channelOptions(options, false);

            range_theta = obj.range_model.initialGuess(range_m, range_residual_m, range_options);
            bearing_theta = obj.bearing_model.initialGuess(range_m, bearing_residual_rad, bearing_options);

            theta_initial = [range_theta(:); bearing_theta(:)];
        end

        function param_labels = parameterLabels(obj)
            range_labels = obj.range_model.parameterLabels("m");
            bearing_labels = obj.bearing_model.parameterLabels("rad");

            param_labels = [
                "range_" + range_labels(:)
                "bearing_" + bearing_labels(:)
            ];
        end

        function [A, b] = inequalityConstraints(obj, range_m, options)
            range_options = obj.channelOptions(options, true);
            bearing_options = obj.channelOptions(options, false);

            [A_range, b_range] = obj.range_model.inequalityConstraints(range_m, range_options);
            [A_bearing, b_bearing] = obj.bearing_model.inequalityConstraints(range_m, bearing_options);

            range_count = obj.range_model.parameterCount();
            bearing_count = obj.bearing_model.parameterCount();

            A = [];
            b = [];

            if ~isempty(A_range)
                A = [A; [A_range, zeros(size(A_range, 1), bearing_count)]]; %#ok<AGROW>
                b = [b; b_range]; %#ok<AGROW>
            end

            if ~isempty(A_bearing)
                A = [A; [zeros(size(A_bearing, 1), range_count), A_bearing]]; %#ok<AGROW>
                b = [b; b_bearing]; %#ok<AGROW>
            end
        end
    end

    methods (Access = private)
        function channel_options = channelOptions(~, options, is_range)
            if is_range
                sigma_floor = options.sigma_floor_range_m;
                variance_floor = options.variance_floor_range_m2;
            else
                sigma_floor = options.sigma_floor_bearing_rad;
                variance_floor = options.variance_floor_bearing_rad2;
            end

            channel_options = struct( ...
                "sigma_floor", sigma_floor, ...
                "variance_floor", variance_floor, ...
                "num_sigma_bins", options.num_sigma_bins);
        end
    end
end
