# Tocaro Blue Simulation Refactor

This repository simulates BlueBoat trajectories, radar measurements, and a parametric MLE solver for estimating radar bias/variance families. The code is organized so you can swap bias/variance model families and optimizers without editing the solver internals.

## Quick start

1. Open MATLAB in the repo root.
2. Run the main script:

```matlab
setup_path();
main;
```

## Repository layout

```
src/
  boats/           BlueBoat dynamics + onboard EKF simulation
  experiments/     Experiment orchestration (BlueBoatExperiment)
  geometry/        Pose2 / Rot2 primitives
  models/          Bias + variance families used in simulation and MLE
  radar/           Radar sensor models + measurement types
  solvers/         Sensor model MLE solver + optimizer adapters
  utils/           Plotting + math utilities
```

## Swap bias/variance families

The solver and the parametric radar model both accept bias/variance model objects:

```matlab
bias_model = LinearBiasModel();          % or QuadraticBiasModel()
variance_model = LinearSigmaVarianceModel(); % or LogLinearVarianceModel()

solver = SensorModelSolver( ...
    "bias_model", bias_model, ...
    "variance_model", variance_model);
```

To run the simulation with the same families:

```matlab
true_radar_model = ParametricRadarSensorModel( ...
    bias_model, variance_model, ...
    "bias_parameters", [...], ...
    "variance_parameters", [...]);
```

## Swap optimizers

The solver uses a pluggable optimizer strategy. To change it, pass a different optimizer instance:

```matlab
solver = SensorModelSolver( ...
    "optimizer", FminsearchOptimizer()); % or FminuncOptimizer(), FminconOptimizer()
```

You can implement a custom optimizer by subclassing `OptimizerBase` and implementing the `optimize` method (e.g., a Cholesky-based Gauss-Newton solver).

## Experiment orchestration

Use `BlueBoatExperiment` to generate trajectories, simulate radar detections, and fit parameters:

```matlab
experiment = BlueBoatExperiment( ...
    "boats", [boat1; boat2], ...
    "waypoints", {wp1, wp2}, ...
    "true_radar_model", true_radar_model, ...
    "prototype_radar_model", prototype_radar_model, ...
    "solver", solver);

results = experiment.run();
```

## Experiment parameters you can change

The default experiment setup lives in `main.m`. The settings below are the most common knobs to turn when you want a different scenario or sensor model. Each item includes a plain-language explanation so you do not need a state-estimation background to get started.

### Scenario + timing

* **Waypoints (`boat*_waypoints_m`)**: A list of target points each boat will try to visit, in meters. Add/remove rows to change the path, or add a new boat with its own waypoint list.
* **Simulation length (`simulation_total_time_sec`)**: How long the world runs, in seconds. Increase to let boats complete longer routes.
* **Simulation time step (`simulation_dt_sec`)**: The physics/estimator update step in seconds. Smaller values give smoother motion but take longer to run.

### Boat motion + onboard sensors

Each `BlueBoat(...)` block configures a vehicle and its onboard sensors. These are the most impactful parameters:

* **Initial pose (`x0`, `y0`, `theta0`)**: Where the boat starts and which way it faces (radians).
* **Speed + turn limits (`nominal_speed`, `yaw_rate_max`)**: Desired forward speed (m/s) and maximum turning rate (rad/s).
* **Waypoint following (`wp_accept_radius`, `heading_kp`, `slow_down_radius`)**: How close the boat must get to call a waypoint “done,” how aggressively it turns toward the waypoint, and when it starts slowing down as it approaches.
* **GPS + heading sensor rates (`gps_period`, `heading_period`)**: How often the simulated GPS and compass update (seconds).
* **GPS + heading noise (`gps_sigma_xy`, `heading_sigma`)**: Measurement uncertainty (meters for GPS, radians for heading). Larger values mean noisier onboard measurements.
* **Odometry noise (`odom_sigma_v`, `odom_sigma_w`)**: Noise on speed (m/s) and yaw-rate (rad/s) estimates produced by the onboard odometry model.
* **Systematic bias (`bias_v`, `bias_w`)**: Constant offsets added to the odometry velocity and yaw rate. Use these to simulate drift or miscalibration.
* **Random seed (`rng_seed`)**: Controls the random number stream so you can reproduce runs.

### Radar + bias/variance models

The radar is a simulated dock-mounted sensor that produces range + bearing measurements:

* **Radar pose (`dock_radar_pose_world`, `radar_mount_in_dock_frame`)**: Where the radar is mounted and how it is oriented in the world frame.
* **Bias/variance family (`bias_model`, `variance_model`)**: The functional form used to model systematic errors and noise growth with range (e.g., linear vs. log-linear).
* **True bias parameters (`true_bias_params`)**: The “ground-truth” coefficients for the bias model used to generate simulated measurements.
* **True variance parameters (`true_variance_params`)**: The “ground-truth” coefficients for the noise model (standard deviations). Larger values mean noisier radar measurements.
* **Variance options (`variance_options`)**: Floors on the minimum allowed uncertainty and the number of bins used to estimate variance vs. range. Use these to prevent zero-variance cases or to smooth noise estimates.
* **Noise toggle (`enable_noise`)**: Set to `false` to generate noise-free radar measurements for debugging.
* **Range limits (`range_min_m`, `range_max_m`)**: The valid operating range of the radar. Measurements outside this range are ignored.
* **Prototype model (`prototype_radar_model`)**: The initial guess for the solver. Adjust these parameters to test solver convergence from different starting points.

### Solver configuration

The solver estimates the bias/variance parameters from the simulated radar data:

* **Minimum predicted range (`min_predicted_range_m`)**: Filters out extremely close targets where bearing/range equations can become unstable.
* **Bearing wrap margin (`bearing_wrap_margin_rad`)**: Controls how the solver handles angle wrapping near ±π to avoid discontinuities.
* **Display (`display`)**: MATLAB optimization display level (e.g., `final`, `iter`).

### Experiment-level switches

* **Fit using estimated poses (`fit_using_estimated_poses`)**: If `true`, the solver uses the boats’ onboard estimated poses; if `false`, it uses the ground-truth poses. This helps study sensitivity to state-estimation error.
* **Simulation duration + waypoints in experiment (`simulation_total_time_sec`, `waypoints`)**: Passed into `BlueBoatExperiment` to define how long to run and what paths to follow.

### Output + plotting

Use `output_options` to control what plots, prints, and files are produced:

* **Plot toggles (`output_options.plots`)**: Enable/disable trajectory plots, covariance ellipses, radar residuals, and sensor-model curves.
* **Printed summaries (`output_options.prints`)**: Enable/disable text summaries like RMSE and fitted parameter tables.
* **Output files (`output_options.output`)**: Turn saving on/off, change the output directory, and decide which artifacts to save (plots, parameters, summaries).
