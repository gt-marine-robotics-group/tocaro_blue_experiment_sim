function display_experiment_results(results, context, options)
%DISPLAY_EXPERIMENT_RESULTS Render plots and print summaries for experiment outputs.
%   results: output from BlueBoatExperiment.run()
%   context: struct with fields boats, waypoints, true_radar_model,
%            prototype_radar_model (optional), bias_model, variance_model
%   options: struct to enable/disable plots and printed summaries

if nargin < 3
    options = struct();
end

options = apply_default_options(options);

output_config = options.output;
log_lines = {};
saved_plots = {};

if output_config.enabled
    if strlength(output_config.directory) == 0
        output_config.directory = fullfile("outputs", "run_" + string(datestr(now, "yyyy-mm-dd_HHMMSS")));
    end
    output_config.directory = char(output_config.directory);
    output_config.plots_dir = fullfile(output_config.directory, "plots");
    mkdir(output_config.directory);
    if output_config.save_plots
        mkdir(output_config.plots_dir);
    end
end

boats = context.boats;
waypoints = context.waypoints;

if ~iscell(waypoints)
    waypoints = {waypoints};
end

num_boats = numel(boats);
if num_boats == 0
    error("Expected at least one boat in context.boats.");
end
if numel(waypoints) ~= num_boats
    error("Waypoints must be a cell array matching the number of boats.");
end

true_radar_model = context.true_radar_model;
bias_model = context.bias_model;
variance_model = context.variance_model;

color_map = lines(max(num_boats, 1));
waypoint_line_styles = ["--", ":", "-.", "-"];
waypoint_markers = ["o", "s", "d", "^", "v", ">", "<", "p", "h"];

sims = results.sims;
boat_xy_true_m = cell(num_boats, 1);
boat_xy_est_m = cell(num_boats, 1);
for boat_index = 1:num_boats
    boat_xy_true_m{boat_index} = sims{boat_index}.ground_truth_xytheta(:, 1:2);
    boat_xy_est_m{boat_index} = sims{boat_index}.est_xytheta(:, 1:2);
end

%% =======================================================================
%  PLOTS: TRAJECTORIES
%  =======================================================================

if options.plots.trajectories_true
    fig_traj_true = figure("Name","Ground truth XY"); hold on; grid on; axis equal;

    legend_entries = cell(2 * num_boats, 1);
    for boat_index = 1:num_boats
        color = color_map(boat_index, :);
        line_style = waypoint_line_styles(mod(boat_index - 1, numel(waypoint_line_styles)) + 1);
        marker_style = waypoint_markers(mod(boat_index - 1, numel(waypoint_markers)) + 1);
        plot(waypoints{boat_index}(:,1), waypoints{boat_index}(:,2), ...
            line_style, "Color", color, "LineWidth", 1.2);
        plot(waypoints{boat_index}(:,1), waypoints{boat_index}(:,2), ...
            marker_style, "Color", color, "MarkerFaceColor", color, "HandleVisibility", "off");

        plot(boat_xy_true_m{boat_index}(:,1), boat_xy_true_m{boat_index}(:,2), ...
            "Color", color, "LineWidth", 1.5);

        legend_entries{2 * boat_index - 1} = boats(boat_index).name + " waypoints";
        legend_entries{2 * boat_index} = boats(boat_index).name + " true";
    end

    xlabel("x [m]"); ylabel("y [m]");
    legend(legend_entries{:}, "Location", "best");
    title("BlueBoat ground-truth trajectories");

    save_figure(fig_traj_true, output_config, "trajectories_true");
end

plot_handle_boats = gobjects(num_boats, 1);

if options.plots.trajectories_estimated
    fig_traj_est = figure("Name","Estimated XY"); hold on; grid on; axis equal;

    legend_entries = cell(2 * num_boats, 1);
    for boat_index = 1:num_boats
        color = color_map(boat_index, :);
        line_style = waypoint_line_styles(mod(boat_index - 1, numel(waypoint_line_styles)) + 1);
        marker_style = waypoint_markers(mod(boat_index - 1, numel(waypoint_markers)) + 1);
        plot(waypoints{boat_index}(:,1), waypoints{boat_index}(:,2), ...
            line_style, "Color", color, "LineWidth", 1.2);
        plot(waypoints{boat_index}(:,1), waypoints{boat_index}(:,2), ...
            marker_style, "Color", color, "MarkerFaceColor", color, "HandleVisibility", "off");

        plot_handle_boats(boat_index) = plot(boat_xy_est_m{boat_index}(:,1), boat_xy_est_m{boat_index}(:,2), ...
            "Color", color, "LineWidth", 1.5);

        legend_entries{2 * boat_index - 1} = boats(boat_index).name + " waypoints";
        legend_entries{2 * boat_index} = boats(boat_index).name + " est";
    end

    xlabel("x [m]"); ylabel("y [m]");
    legend(legend_entries{:}, "Location", "best");
    title("BlueBoat estimated trajectories (GPS+heading updates)");

    save_figure(fig_traj_est, output_config, "trajectories_estimated");
end

%% --- Covariance ellipses along estimated trajectories ---
if options.plots.covariance_ellipses
    num_ellipses = 25;
    ellipse_sigma_multiplier = 2; % 2-sigma
    plotted_any = false;

    for boat_index = 1:num_boats
        if ~isfield(sims{boat_index}, "covariance_history")
            warning("No covariance_history found for boat %d. Skipping covariance ellipse plots for this boat.", boat_index);
            continue;
        end
        cov_history = sims{boat_index}.covariance_history;
        est_xy = boat_xy_est_m{boat_index};
        indices = unique(round(linspace(1, size(est_xy, 1), num_ellipses)));
        for index = indices
            position_covariance = cov_history(1:2, 1:2, index);
            ellipse_handle = plot_cov_ellipse_2d(est_xy(index,1), est_xy(index,2), position_covariance, ellipse_sigma_multiplier);
            if ~isempty(ellipse_handle); ellipse_handle.LineWidth = 1.0; end
        end
        plotted_any = true;
    end

    if plotted_any
        for boat_index = 1:num_boats
            if ~isempty(plot_handle_boats(boat_index))
                uistack(plot_handle_boats(boat_index), "top");
            end
        end
    end
end

%% =======================================================================
%  TRAJECTORY RMSE
%  =======================================================================

if options.prints.trajectory_rmse
    log_printf("\n--------- RMSE (Trajectory) ---------\n");
    for boat_index = 1:num_boats
        position_error_m = boat_xy_est_m{boat_index} - boat_xy_true_m{boat_index};
        rmse_xy_m = sqrt(mean(sum(position_error_m.^2, 2)));
        log_printf("%s RMSE_xy = %.3f m\n", boats(boat_index).name, rmse_xy_m);
    end
end

%% =======================================================================
%  RADAR SIMULATION OUTPUTS
%  =======================================================================

boat_in_radar_true = results.boat_true;
boat_in_radar_est = results.boat_est;
boat_meas_actual = results.measurements_actual;
boat_meas_pred = results.measurements_prototype;

boat_residual = cell(num_boats, 1);
boat_whiten_model = cell(num_boats, 1);
boat_whiten_true = cell(num_boats, 1);
boat_rmse_model_whitened = zeros(num_boats, 1);
boat_rmse_true_whitened = zeros(num_boats, 1);

for boat_index = 1:num_boats
    boat_residual{boat_index} = boat_meas_pred{boat_index} - boat_meas_actual{boat_index};
    boat_whiten_model{boat_index} = RadarMeasurement.whitenResidualUsingSigmas( ...
        boat_meas_pred{boat_index}, boat_residual{boat_index});
    boat_whiten_true{boat_index} = RadarMeasurement.whitenResidualUsingSigmas( ...
        boat_meas_actual{boat_index}, boat_residual{boat_index});
    boat_rmse_model_whitened(boat_index) = sqrt(mean(sum(boat_whiten_model{boat_index}.^2, 2)));
    boat_rmse_true_whitened(boat_index) = sqrt(mean(sum(boat_whiten_true{boat_index}.^2, 2)));
end

if options.prints.radar_rmse
    log_printf("\n--------- RADAR (Dock-mounted) ---------\n");
    for boat_index = 1:num_boats
        log_printf("%s whitened RMSE (prototype sigmas): %.3f\n", ...
            boats(boat_index).name, boat_rmse_model_whitened(boat_index));
    end
    for boat_index = 1:num_boats
        log_printf("%s whitened RMSE (TRUE sigmas):      %.3f\n", ...
            boats(boat_index).name, boat_rmse_true_whitened(boat_index));
    end
end

%% =======================================================================
%  SENSOR MODEL SOLVER OUTPUTS
%  =======================================================================

fit_result = results.fit_result;
identified_radar_model = results.identified_model;

boat_meas_fit = cell(num_boats, 1);
for boat_index = 1:num_boats
    boat_meas_fit{boat_index} = identified_radar_model.measure(boat_in_radar_est{boat_index});
end

%% =======================================================================
%  PARAMETER COMPARISON (NOW DIRECTLY COMPARABLE)
%  =======================================================================

theta_true_equiv = [true_radar_model.bias_parameters(:); true_radar_model.variance_parameters(:)];
parameter_names = [fit_result.bias_parameter_labels; fit_result.variance_parameter_labels];

if options.prints.sensor_model_theta
    log_printf("\n--------- SENSOR MODEL SOLVER (Theta) ---------\n");
    log_printf("Fit used poses: %s\n", ternary(results.fit_using_estimated_poses, "ESTIMATED", "TRUE"));
    theta_text = evalc("print_theta_comparison(theta_true_equiv, fit_result.theta_vector, parameter_names);");
    log_print_multiline(theta_text);
end

%% =======================================================================
%  EVALUATE IMPROVEMENT VS PROTOTYPE
%  =======================================================================

boat_residual_fit = cell(num_boats, 1);
boat_whiten_fit_model = cell(num_boats, 1);
boat_whiten_fit_true = cell(num_boats, 1);
boat_rmse_fit_model_whitened = zeros(num_boats, 1);
boat_rmse_fit_true_whitened = zeros(num_boats, 1);

for boat_index = 1:num_boats
    boat_residual_fit{boat_index} = boat_meas_fit{boat_index} - boat_meas_actual{boat_index};
    boat_whiten_fit_model{boat_index} = RadarMeasurement.whitenResidualUsingSigmas( ...
        boat_meas_fit{boat_index}, boat_residual_fit{boat_index});
    boat_whiten_fit_true{boat_index} = RadarMeasurement.whitenResidualUsingSigmas( ...
        boat_meas_actual{boat_index}, boat_residual_fit{boat_index});
    boat_rmse_fit_model_whitened(boat_index) = sqrt(mean(sum(boat_whiten_fit_model{boat_index}.^2, 2)));
    boat_rmse_fit_true_whitened(boat_index) = sqrt(mean(sum(boat_whiten_fit_true{boat_index}.^2, 2)));
end

if options.prints.prototype_vs_fitted
    log_printf("\n--------- RADAR (Prototype vs Fitted) ---------\n");
    for boat_index = 1:num_boats
        log_printf("%s whitened RMSE (prototype sigmas): %.3f\n", ...
            boats(boat_index).name, boat_rmse_model_whitened(boat_index));
        log_printf("%s whitened RMSE (fitted sigmas):    %.3f\n", ...
            boats(boat_index).name, boat_rmse_fit_model_whitened(boat_index));
        log_printf("%s whitened RMSE (TRUE sigmas):      %.3f (prototype residuals)\n", ...
            boats(boat_index).name, boat_rmse_true_whitened(boat_index));
        log_printf("%s whitened RMSE (TRUE sigmas):      %.3f (fitted residuals)\n\n", ...
            boats(boat_index).name, boat_rmse_fit_true_whitened(boat_index));
    end
end

%% =======================================================================
%  TRUE vs ESTIMATED FUNCTION FORMS + PLOTS
%  =======================================================================

poses_for_fit = results.boat_true;
if results.fit_using_estimated_poses
    poses_for_fit = results.boat_est;
end

predicted_ranges_all_m = [];
for boat_index = 1:num_boats
    predicted_ranges_all_m = [predicted_ranges_all_m; ...
        hypot([poses_for_fit{boat_index}.x].', [poses_for_fit{boat_index}.y].')];
end
predicted_ranges_all_m = predicted_ranges_all_m(:);

range_grid_m = linspace(min(predicted_ranges_all_m), max(predicted_ranges_all_m), 300).';

if options.prints.sensor_model_functions
    log_printf("\n--------- SENSOR MODEL: TRUE vs ESTIMATED (FUNCTION FORMS) ---------\n");
    function_text = evalc("print_true_vs_estimated_functions(true_radar_model, fit_result);");
    log_print_multiline(function_text);
end

[true_range_bias_grid_m, true_bearing_bias_grid_rad] = bias_model.evaluate(range_grid_m, true_radar_model.bias_parameters);
[fit_range_bias_grid_m, fit_bearing_bias_grid_rad] = bias_model.evaluate(range_grid_m, fit_result.bias_parameters);

variance_options = fit_result.options_used;
[true_range_var_grid, true_bearing_var_grid] = variance_model.evaluate( ...
    range_grid_m, true_radar_model.variance_parameters, variance_options);
[fit_range_var_grid, fit_bearing_var_grid] = variance_model.evaluate( ...
    range_grid_m, fit_result.variance_parameters, variance_options);

true_range_sigma_grid_m = sqrt(true_range_var_grid);
true_bearing_sigma_grid_rad = sqrt(true_bearing_var_grid);

fit_range_sigma_grid_m = sqrt(fit_range_var_grid);
fit_bearing_sigma_grid_rad = sqrt(fit_bearing_var_grid);

if options.plots.sensor_model_bias_sigma
    fig_bias = figure("Name", "Sensor Model: Bias vs Range");
    subplot(2,1,1); hold on; grid on;
    plot(range_grid_m, true_range_bias_grid_m, "LineWidth", 1.5);
    plot(range_grid_m, fit_range_bias_grid_m, "LineWidth", 1.5);
    xlabel("Predicted range \hat{\rho} [m]"); ylabel("Range bias [m]");
    legend("True", "Fitted", "Location", "best");
    title("Range bias vs predicted range");

    subplot(2,1,2); hold on; grid on;
    plot(range_grid_m, rad2deg(true_bearing_bias_grid_rad), "LineWidth", 1.5);
    plot(range_grid_m, rad2deg(fit_bearing_bias_grid_rad), "LineWidth", 1.5);
    xlabel("Predicted range \hat{\rho} [m]"); ylabel("Bearing bias [deg]");
    legend("True", "Fitted", "Location", "best");
    title("Bearing bias vs predicted range");

    save_figure(fig_bias, output_config, "sensor_model_bias_vs_range");

    fig_sigma = figure("Name", "Sensor Model: Sigma vs Range (Linear family)");
    subplot(2,1,1); hold on; grid on;
    plot(range_grid_m, true_range_sigma_grid_m, "LineWidth", 1.5);
    plot(range_grid_m, fit_range_sigma_grid_m, "LineWidth", 1.5);
    xlabel("Predicted range \hat{\rho} [m]"); ylabel("\sigma_\rho [m]");
    legend("True", "Fitted", "Location", "best");
    title("Range sigma vs predicted range");

    subplot(2,1,2); hold on; grid on;
    plot(range_grid_m, rad2deg(true_bearing_sigma_grid_rad), "LineWidth", 1.5);
    plot(range_grid_m, rad2deg(fit_bearing_sigma_grid_rad), "LineWidth", 1.5);
    xlabel("Predicted range \hat{\rho} [m]"); ylabel("\sigma_\phi [deg]");
    legend("True", "Fitted", "Location", "best");
    title("Bearing sigma vs predicted range");

    save_figure(fig_sigma, output_config, "sensor_model_sigma_vs_range");
end

%% =======================================================================
%  RADAR PLOTS (detections + residuals)
%  =======================================================================

boat_det_xy_m = cell(num_boats, 1);
boat_det_range_m = cell(num_boats, 1);
boat_true_xy_radar_m = cell(num_boats, 1);
for boat_index = 1:num_boats
    [boat_det_xy_m{boat_index}, boat_det_range_m{boat_index}, ~] = radar_meas_to_xy(boat_meas_actual{boat_index});
    boat_true_xy_radar_m{boat_index} = [[boat_in_radar_true{boat_index}.x].', [boat_in_radar_true{boat_index}.y].'];
end

if options.plots.radar_detections
    fig_radar = figure("Name","Dock Radar Frame"); hold on; grid on; axis equal;
    plot(0,0,"k^","MarkerFaceColor","k"); % radar location
    legend_entries = cell(1 + 2 * num_boats, 1);
    legend_entries{1} = "Radar (dock)";
    entry_index = 2;
    for boat_index = 1:num_boats
        color = color_map(boat_index, :);
        plot(boat_true_xy_radar_m{boat_index}(:,1), boat_true_xy_radar_m{boat_index}(:,2), ...
            "Color", color, "LineWidth", 1.5);
        scatter(boat_det_xy_m{boat_index}(:,1), boat_det_xy_m{boat_index}(:,2), 12, ...
            "filled", "MarkerFaceAlpha", 0.35, "MarkerFaceColor", color, "MarkerEdgeColor", color);
        legend_entries{entry_index} = boats(boat_index).name + " true (radar frame)";
        legend_entries{entry_index + 1} = boats(boat_index).name + " detections";
        entry_index = entry_index + 2;
    end
    xlabel("x_r [m]"); ylabel("y_r [m]");
    legend(legend_entries{:}, "Location","best");
    title("Dock-mounted radar detections in radar frame");

    save_figure(fig_radar, output_config, "radar_detections");
end

if options.plots.radar_residuals
    fig_residuals = figure("Name","Radar residuals vs range");
    subplot(2,1,1); hold on; grid on;
    legend_entries = cell(num_boats, 1);
    for boat_index = 1:num_boats
        drange_m = boat_residual{boat_index}(:,2);
        plot(boat_det_range_m{boat_index}, drange_m, ".");
        legend_entries{boat_index} = boats(boat_index).name;
    end
    xlabel("Range [m]"); ylabel("Range residual [m]");
    legend(legend_entries{:}, "Location","best");
    title("Prototype - Actual residuals");

    subplot(2,1,2); hold on; grid on;
    for boat_index = 1:num_boats
        dtheta_rad = boat_residual{boat_index}(:,1);
        plot(boat_det_range_m{boat_index}, rad2deg(dtheta_rad), ".");
    end
    xlabel("Range [m]"); ylabel("Bearing residual [deg]");
    legend(legend_entries{:}, "Location","best");

    save_figure(fig_residuals, output_config, "radar_residuals");
end

if options.prints.done_message
    log_printf("Done. Figures generated.\n");
end

if output_config.enabled
    params = build_simulation_parameters(context, results, options);
    if output_config.save_parameters
        params_path = fullfile(output_config.directory, "simulation_parameters.json");
        write_json(params_path, params);
    end

    if output_config.save_prints
        results_path = fullfile(output_config.directory, "results.txt");
        write_text_lines(results_path, log_lines);
    end

    if output_config.save_summary
        summary_path = fullfile(output_config.directory, "summary.md");
        write_summary(summary_path, params, log_lines, saved_plots, output_config);
    end
end
end

function options = apply_default_options(options)
default_options = struct( ...
    "plots", struct( ...
        "trajectories_true", true, ...
        "trajectories_estimated", true, ...
        "covariance_ellipses", true, ...
        "sensor_model_bias_sigma", true, ...
        "radar_detections", true, ...
        "radar_residuals", true), ...
    "prints", struct( ...
        "trajectory_rmse", true, ...
        "radar_rmse", true, ...
        "sensor_model_theta", true, ...
        "prototype_vs_fitted", true, ...
        "sensor_model_functions", true, ...
        "done_message", true), ...
    "output", struct( ...
        "enabled", false, ...
        "directory", "", ...
        "save_plots", true, ...
        "save_parameters", true, ...
        "save_prints", true, ...
        "save_summary", true));

options = merge_structs(default_options, options);
options = apply_plot_flags(options);
end

function options = apply_plot_flags(options)
plot_fields = fieldnames(options.plots);
for idx = 1:numel(plot_fields)
    plot_field = plot_fields{idx};
    legacy_flag = "show_" + plot_field;
    if isfield(options, legacy_flag)
        options.plots.(plot_field) = options.(legacy_flag);
    end
end
end

function merged = merge_structs(base, override)
merged = base;
if isempty(override)
    return;
end

override_fields = fieldnames(override);
for idx = 1:numel(override_fields)
    field = override_fields{idx};
    if isfield(base, field) && isstruct(base.(field)) && isstruct(override.(field))
        merged.(field) = merge_structs(base.(field), override.(field));
    else
        merged.(field) = override.(field);
    end
end
end

function log_printf(fmt, varargin)
persistent log_lines_ref
text = sprintf(fmt, varargin{:});
fprintf("%s", text);
log_lines_ref = append_log_lines(log_lines_ref, text);
end

function log_print_multiline(text)
if isempty(text)
    return;
end
lines = splitlines(string(text));
if strlength(lines(end)) == 0
    lines(end) = [];
end
for idx = 1:numel(lines)
    log_printf("%s\n", lines(idx));
end
end

function lines = append_log_lines(lines, text)
new_lines = splitlines(string(text));
if strlength(new_lines(end)) == 0
    new_lines(end) = [];
end
lines = [lines; cellstr(new_lines)];
end

function save_figure(fig_handle, output_config, file_stem)
if ~output_config.enabled || ~output_config.save_plots
    return;
end
if isempty(fig_handle) || ~ishandle(fig_handle)
    return;
end
file_stem = sanitize_filename(file_stem);
file_path = fullfile(output_config.plots_dir, file_stem + ".png");
try
    exportgraphics(fig_handle, file_path, "Resolution", 150);
catch
    saveas(fig_handle, file_path);
end
saved_plots = evalin("caller", "saved_plots");
saved_plots{end+1} = file_path; %#ok<AGROW>
assignin("caller", "saved_plots", saved_plots);
end

function filename = sanitize_filename(name)
name = regexprep(string(name), "[^A-Za-z0-9_-]+", "_");
name = regexprep(name, "_+", "_");
name = strip(name, "_");
if strlength(name) == 0
    name = "figure";
end
filename = name;
end

function params = build_simulation_parameters(context, results, options)
params = struct();
params.timestamp = string(datetime("now"));
if isfield(context, "simulation")
    params.simulation = context.simulation;
end

params.fit_using_estimated_poses = results.fit_using_estimated_poses;
params.waypoints = context.waypoints;

boats = context.boats;
boat_params = cell(numel(boats), 1);
for idx = 1:numel(boats)
    boat_params{idx} = boat_to_struct(boats(idx));
end
params.boats = boat_params;

params.radar = struct();
if isfield(context, "radar_pose_world")
    params.radar.pose_world = pose_to_struct(context.radar_pose_world);
end
params.radar.true_model = radar_model_to_struct(context.true_radar_model);
if isfield(context, "prototype_radar_model") && ~isempty(context.prototype_radar_model)
    params.radar.prototype_model = radar_model_to_struct(context.prototype_radar_model);
end
params.radar.bias_model = class(context.bias_model);
params.radar.variance_model = class(context.variance_model);

if isfield(context, "solver")
    params.solver = solver_to_struct(context.solver);
end

params.output = options.output;
end

function boat_struct = boat_to_struct(boat)
boat_struct = struct( ...
    "name", boat.name, ...
    "x0", boat.x0, ...
    "y0", boat.y0, ...
    "theta0", boat.theta0, ...
    "dt", boat.dt, ...
    "nominal_speed", boat.nominal_speed, ...
    "yaw_rate_max", boat.yaw_rate_max, ...
    "accel_max", boat.accel_max, ...
    "wp_accept_radius", boat.wp_accept_radius, ...
    "heading_kp", boat.heading_kp, ...
    "slow_down_radius", boat.slow_down_radius, ...
    "gps_period", boat.gps_period, ...
    "heading_period", boat.heading_period, ...
    "gps_sigma_xy", boat.gps_sigma_xy, ...
    "heading_sigma", boat.heading_sigma, ...
    "odom_sigma_v", boat.odom_sigma_v, ...
    "odom_sigma_w", boat.odom_sigma_w, ...
    "bias_v", boat.bias_v, ...
    "bias_w", boat.bias_w, ...
    "rng_seed", boat.rng_seed);
end

function radar_struct = radar_model_to_struct(radar_model)
radar_struct = struct( ...
    "bias_parameters", radar_model.bias_parameters, ...
    "variance_parameters", radar_model.variance_parameters, ...
    "enable_noise", radar_model.enable_noise, ...
    "variance_options", radar_model.variance_options, ...
    "range_min_m", radar_model.range_min_m, ...
    "range_max_m", radar_model.range_max_m);
end

function pose_struct = pose_to_struct(pose)
pose_struct = struct("x", pose.x, "y", pose.y, "theta", pose.theta);
end

function solver_struct = solver_to_struct(solver)
solver_struct = struct( ...
    "bias_model", class(solver.bias_model), ...
    "variance_model", class(solver.variance_model), ...
    "optimizer", class(solver.optimizer), ...
    "options", solver.options);
end

function write_json(path, data)
try
    json_text = jsonencode(data, "PrettyPrint", true);
catch
    json_text = jsonencode(data);
end
fid = fopen(path, "w");
if fid < 0
    warning("Failed to write JSON file: %s", path);
    return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s\n", json_text);
end

function write_text_lines(path, lines)
fid = fopen(path, "w");
if fid < 0
    warning("Failed to write text file: %s", path);
    return;
end
cleanup = onCleanup(@() fclose(fid));
for idx = 1:numel(lines)
    fprintf(fid, "%s\n", lines{idx});
end
end

function write_summary(path, params, log_lines, saved_plots, output_config)
fid = fopen(path, "w");
if fid < 0
    warning("Failed to write summary file: %s", path);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Simulation Summary\n\n");
fprintf(fid, "- Timestamp: %s\n", params.timestamp);
fprintf(fid, "- Output directory: `%s`\n\n", output_config.directory);

fprintf(fid, "## Simulation Parameters\n\n");
fprintf(fid, "```\n");
try
    params_text = jsonencode(params, "PrettyPrint", true);
catch
    params_text = jsonencode(params);
end
fprintf(fid, "%s\n", params_text);
fprintf(fid, "```\n\n");

fprintf(fid, "## Printed Results\n\n");
if isempty(log_lines)
    fprintf(fid, "_No printed results were captured._\n\n");
else
    fprintf(fid, "```\n");
    for idx = 1:numel(log_lines)
        fprintf(fid, "%s\n", log_lines{idx});
    end
    fprintf(fid, "```\n\n");
end

fprintf(fid, "## Saved Plots\n\n");
if isempty(saved_plots)
    fprintf(fid, "_No plots were saved._\n");
else
    for idx = 1:numel(saved_plots)
        [~, name, ext] = fileparts(saved_plots{idx});
        fprintf(fid, "- `%s`\n", fullfile("plots", name + ext));
    end
end
end
