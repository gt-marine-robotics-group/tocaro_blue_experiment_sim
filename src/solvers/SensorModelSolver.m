classdef SensorModelSolver
    %SensorModelSolver Modular MLE solver for radar bias + variance models.

    properties
        bias_model RadarBiasModel = LinearBiasModel()
        variance_model RadarVarianceModel = LinearSigmaVarianceModel()
        optimizer OptimizerBase = FminuncOptimizer()
        options struct
    end

    methods
        function obj = SensorModelSolver(varargin)
            options = SensorModelSolver.parseOptions(varargin{:});

            obj.bias_model = options.bias_model;
            obj.variance_model = options.variance_model;
            obj.optimizer = options.optimizer;
            obj.options = options;
        end

        function fit_result = solve(obj, target_poses_in_radar_frame, radar_measurements, varargin)
            options = obj.options;
            if ~isempty(varargin)
                override = SensorModelSolver.parseOptions(varargin{:});
                options = SensorModelSolver.mergeOptions(options, override);
            end

            bias_model = options.bias_model;
            variance_model = options.variance_model;
            optimizer = options.optimizer;

            [predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad] = ...
                SensorModelSolver.flattenDataset(target_poses_in_radar_frame, radar_measurements);

            raw_bearing_residual = wrap_to_pi(measured_bearing_rad - predicted_bearing_rad);

            is_valid_sample = isfinite(predicted_range_m) & isfinite(predicted_bearing_rad) & ...
                              isfinite(measured_range_m) & isfinite(measured_bearing_rad);

            is_valid_sample = is_valid_sample & ...
                              predicted_range_m >= options.min_predicted_range_m & ...
                              predicted_range_m <= options.max_predicted_range_m;

            if options.bearing_wrap_margin_rad > 0
                is_valid_sample = is_valid_sample & ...
                    abs(raw_bearing_residual) <= (pi - options.bearing_wrap_margin_rad);
            end

            predicted_range_m = predicted_range_m(is_valid_sample);
            predicted_bearing_rad = predicted_bearing_rad(is_valid_sample);
            measured_range_m = measured_range_m(is_valid_sample);
            measured_bearing_rad = measured_bearing_rad(is_valid_sample);

            if numel(predicted_range_m) < 10
                error("SensorModelSolver:TooFewSamples", ...
                    "Too few valid samples after gating (%d).", numel(predicted_range_m));
            end

            theta_initial = SensorModelSolver.initializeTheta( ...
                bias_model, variance_model, ...
                predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad, options);

            [A, b] = variance_model.inequalityConstraints(predicted_range_m, options);

            options.A = A;
            options.b = b;

            objective_handle = @(theta_vector) SensorModelSolver.nllObjective( ...
                theta_vector, bias_model, variance_model, ...
                predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad, options);

            if options.use_analytic_gradient
                gradient_available = SensorModelSolver.supportsAnalyticGradient(bias_model, variance_model);
                if ~gradient_available
                    warning("SensorModelSolver:GradientUnavailable", ...
                        "Analytic gradient not available for selected models. Disabling analytic gradients.");
                    options.use_analytic_gradient = false;
                end
            end

            [theta_opt, final_objective_value, exitflag, optimizer_output] = ...
                optimizer.optimize(objective_handle, theta_initial(:), options);

            fit_result = SensorModelSolver.packageResults( ...
                theta_opt, final_objective_value, exitflag, optimizer_output, ...
                predicted_range_m, options, bias_model, variance_model);
        end
    end

    methods (Static)
        function fit_result = fit(target_poses_in_radar_frame, radar_measurements, varargin)
            solver = SensorModelSolver(varargin{:});
            fit_result = solver.solve(target_poses_in_radar_frame, radar_measurements);
        end
    end

    methods (Static, Access = private)
        function options = parseOptions(varargin)
            input_parser = inputParser;
            input_parser.FunctionName = "SensorModelSolver";

            addParameter(input_parser, "min_predicted_range_m", 1.0, @(v) isnumeric(v) && isscalar(v) && v >= 0);
            addParameter(input_parser, "max_predicted_range_m", inf, @(v) isnumeric(v) && isscalar(v) && v > 0);
            addParameter(input_parser, "bearing_wrap_margin_rad", deg2rad(0.5), @(v) isnumeric(v) && isscalar(v) && v >= 0);
            addParameter(input_parser, "num_sigma_bins", 30, @(v) isnumeric(v) && isscalar(v) && v >= 1);

            addParameter(input_parser, "sigma_floor_range_m", 1e-4, @(v) isnumeric(v) && isscalar(v) && v > 0);
            addParameter(input_parser, "sigma_floor_bearing_rad", 1e-6, @(v) isnumeric(v) && isscalar(v) && v > 0);
            addParameter(input_parser, "variance_floor_range_m2", 0.0, @(v) isnumeric(v) && isscalar(v) && v >= 0);
            addParameter(input_parser, "variance_floor_bearing_rad2", 0.0, @(v) isnumeric(v) && isscalar(v) && v >= 0);

            addParameter(input_parser, "use_analytic_gradient", true, @(v) islogical(v) && isscalar(v));
            addParameter(input_parser, "display", "none", @(s) isstring(s) || ischar(s));
            addParameter(input_parser, "max_iterations", 400, @(v) isnumeric(v) && isscalar(v) && v >= 1);

            addParameter(input_parser, "bias_model", LinearBiasModel(), @(v) isa(v, "RadarBiasModel"));
            addParameter(input_parser, "variance_model", LinearSigmaVarianceModel(), @(v) isa(v, "RadarVarianceModel"));
            addParameter(input_parser, "optimizer", SensorModelSolver.defaultOptimizer(), @(v) isa(v, "OptimizerBase"));

            parse(input_parser, varargin{:});
            options = input_parser.Results;

            options.display = string(options.display);
            if ~any(options.display == ["none", "iter", "final", "off"])
                error("SensorModelSolver:BadOption", "display must be one of: none, iter, final, off.");
            end
            if options.display == "off"
                options.display = "none";
            end
        end

        function optimizer = defaultOptimizer()
            if exist("fminunc", "file") == 2
                optimizer = FminuncOptimizer();
            else
                optimizer = FminsearchOptimizer();
            end
        end

        function options = mergeOptions(base, override)
            options = base;
            override_fields = fieldnames(override);
            for idx = 1:numel(override_fields)
                field = override_fields{idx};
                options.(field) = override.(field);
            end
        end

        function theta_initial = initializeTheta(bias_model, variance_model, predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad, options)
            bias_init = bias_model.initialGuess(predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad);

            [range_bias_init, bearing_bias_init] = bias_model.evaluate(predicted_range_m, bias_init);
            range_residual_m = measured_range_m - predicted_range_m - range_bias_init;
            bearing_residual_rad = wrap_to_pi(measured_bearing_rad - predicted_bearing_rad - bearing_bias_init);

            variance_init = variance_model.initialGuess(predicted_range_m, range_residual_m, bearing_residual_rad, options);
            theta_initial = [bias_init(:); variance_init(:)];
        end

        function [nll_value, nll_gradient] = nllObjective(theta_vector, bias_model, variance_model, predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad, options)
            theta_vector = theta_vector(:);
            bias_param_count = bias_model.parameterCount();
            variance_param_count = variance_model.parameterCount();

            if numel(theta_vector) ~= bias_param_count + variance_param_count
                error("SensorModelSolver:ThetaSize", "Theta must be %dx1.", bias_param_count + variance_param_count);
            end

            bias_theta = theta_vector(1:bias_param_count);
            variance_theta = theta_vector(bias_param_count+1:end);

            [range_bias_m, bearing_bias_rad] = bias_model.evaluate(predicted_range_m, bias_theta);
            range_residual_m = measured_range_m - predicted_range_m - range_bias_m;
            bearing_residual_rad = wrap_to_pi(measured_bearing_rad - predicted_bearing_rad - bearing_bias_rad);

            [range_variance_m2, bearing_variance_rad2] = variance_model.evaluate(predicted_range_m, variance_theta, options);

            nll_terms = log(range_variance_m2 .* bearing_variance_rad2) + ...
                        (range_residual_m.^2) ./ range_variance_m2 + ...
                        (bearing_residual_rad.^2) ./ bearing_variance_rad2;
            nll_value = sum(nll_terms);

            if nargout < 2
                return;
            end

            if ~options.use_analytic_gradient
                nll_gradient = SensorModelSolver.numericGradient(@(theta) SensorModelSolver.nllObjective( ...
                    theta, bias_model, variance_model, predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad, options), ...
                    theta_vector);
                return;
            end

            [range_bias_jacobian, bearing_bias_jacobian] = bias_model.jacobian(predicted_range_m, bias_theta);
            [range_var_jacobian, bearing_var_jacobian] = variance_model.jacobian(predicted_range_m, variance_theta, options);

            range_factor = (1 ./ range_variance_m2) - (range_residual_m.^2) ./ (range_variance_m2.^2);
            bearing_factor = (1 ./ bearing_variance_rad2) - (bearing_residual_rad.^2) ./ (bearing_variance_rad2.^2);

            grad_bias = -2 .* (range_bias_jacobian.' * (range_residual_m ./ range_variance_m2)) ...
                        -2 .* (bearing_bias_jacobian.' * (bearing_residual_rad ./ bearing_variance_rad2));

            grad_variance = range_var_jacobian.' * range_factor + bearing_var_jacobian.' * bearing_factor;

            nll_gradient = [grad_bias; grad_variance];
        end

        function gradient = numericGradient(objective_handle, theta_vector)
            step_size = 1e-6;
            gradient = zeros(size(theta_vector));
            for idx = 1:numel(theta_vector)
                delta = zeros(size(theta_vector));
                delta(idx) = step_size;
                gradient(idx) = (objective_handle(theta_vector + delta) - objective_handle(theta_vector - delta)) / (2 * step_size);
            end
        end

        function supports = supportsAnalyticGradient(bias_model, variance_model)
            supports = ~(isempty(bias_model) || isempty(variance_model));
        end

        function fit_result = packageResults(theta_opt, final_objective_value, exitflag, optimizer_output, predicted_range_m, options, bias_model, variance_model)
            bias_param_count = bias_model.parameterCount();
            variance_param_count = variance_model.parameterCount();
            bias_theta = theta_opt(1:bias_param_count);
            variance_theta = theta_opt(bias_param_count+1:bias_param_count+variance_param_count);

            fit_result = struct();
            fit_result.theta_vector = theta_opt(:);
            fit_result.objective_value = final_objective_value;
            fit_result.exitflag = exitflag;
            fit_result.output = optimizer_output;

            dataset_summary = struct();
            dataset_summary.num_samples = numel(predicted_range_m);
            dataset_summary.predicted_range_min_m = min(predicted_range_m);
            dataset_summary.predicted_range_max_m = max(predicted_range_m);
            dataset_summary.predicted_range_mean_m = mean(predicted_range_m);
            dataset_summary.predicted_range_std_m = std(predicted_range_m);
            fit_result.dataset_summary = dataset_summary;

            fit_result.options_used = options;
            fit_result.bias_parameters = bias_theta;
            fit_result.variance_parameters = variance_theta;
            fit_result.bias_parameter_labels = bias_model.parameterLabels();
            fit_result.variance_parameter_labels = variance_model.parameterLabels();
        end

        function [predicted_range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad] = flattenDataset(target_poses_in_radar_frame, radar_measurements)
            if iscell(target_poses_in_radar_frame)
                if ~iscell(radar_measurements)
                    error("SensorModelSolver:InputMismatch", ...
                        "If target_poses_in_radar_frame is a cell array, radar_measurements must be a cell array too.");
                end
                if numel(target_poses_in_radar_frame) ~= numel(radar_measurements)
                    error("SensorModelSolver:InputMismatch", ...
                        "target_poses_in_radar_frame and radar_measurements must have the same number of cells.");
                end

                predicted_range_m = [];
                predicted_bearing_rad = [];
                measured_range_m = [];
                measured_bearing_rad = [];

                for series_index = 1:numel(target_poses_in_radar_frame)
                    [pr, pb, mr, mb] = SensorModelSolver.flattenDataset( ...
                        target_poses_in_radar_frame{series_index}, radar_measurements{series_index});

                    predicted_range_m = [predicted_range_m; pr]; %#ok<AGROW>
                    predicted_bearing_rad = [predicted_bearing_rad; pb]; %#ok<AGROW>
                    measured_range_m = [measured_range_m; mr]; %#ok<AGROW>
                    measured_bearing_rad = [measured_bearing_rad; mb]; %#ok<AGROW>
                end
                return;
            end

            if ~isa(target_poses_in_radar_frame, "Pose2")
                error("SensorModelSolver:BadInput", ...
                    "target_poses_in_radar_frame must be Pose2 or a cell array of Pose2 vectors.");
            end
            if ~isa(radar_measurements, "RadarMeasurement")
                error("SensorModelSolver:BadInput", ...
                    "radar_measurements must be RadarMeasurement or a cell array of RadarMeasurement vectors.");
            end
            if numel(target_poses_in_radar_frame) ~= numel(radar_measurements)
                error("SensorModelSolver:LengthMismatch", ...
                    "Pose2 and RadarMeasurement vectors must have the same length (time-aligned). Got %d vs %d.", ...
                    numel(target_poses_in_radar_frame), numel(radar_measurements));
            end

            x_positions_m = reshape([target_poses_in_radar_frame.x], [], 1);
            y_positions_m = reshape([target_poses_in_radar_frame.y], [], 1);

            predicted_range_m = hypot(x_positions_m, y_positions_m);
            predicted_bearing_rad = atan2(y_positions_m, x_positions_m);

            measured_range_m = reshape([radar_measurements.range], [], 1);
            measured_bearing_rad = reshape([radar_measurements.theta], [], 1);
        end
    end
end
