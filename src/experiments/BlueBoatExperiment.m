classdef BlueBoatExperiment
    %BlueBoatExperiment Orchestrates BlueBoat + radar experiments.

    properties
        boats (:,1) BlueBoat = BlueBoat.empty
        waypoints (:,1) cell = {}
        simulation_total_time_sec (1,1) double {mustBePositive} = 200

        radar_pose_world Pose2 = Pose2(0.0, 0.0, 0.0)
        true_radar_model RadarSensorModel = SimpleSensorModel()
        prototype_radar_model RadarSensorModel = SimpleSensorModel()

        solver SensorModelSolver = SensorModelSolver()
        fit_using_estimated_poses (1,1) logical = false

        radar_rng_seed (1,1) double = 100
    end

    methods
        function obj = BlueBoatExperiment(varargin)
            if ~isempty(varargin)
                if mod(numel(varargin), 2) ~= 0
                    error("BlueBoatExperiment:ConstructorArgs", "Constructor expects name-value pairs.");
                end
                for arg_index = 1:2:numel(varargin)
                    property_name = varargin{arg_index};
                    property_value = varargin{arg_index + 1};
                    if ~isprop(obj, property_name)
                        error("BlueBoatExperiment:UnknownProperty", ...
                            "Unknown property: %s", string(property_name));
                    end
                    obj.(property_name) = property_value;
                end
            end
        end

        function results = run(obj)
            num_boats = numel(obj.boats);
            if num_boats == 0
                error("BlueBoatExperiment:NoBoats", "At least one BlueBoat must be provided.");
            end
            if numel(obj.waypoints) ~= num_boats
                error("BlueBoatExperiment:WaypointMismatch", "Waypoints must be a cell array matching number of boats.");
            end

            sims = cell(num_boats, 1);
            for boat_index = 1:num_boats
                sims{boat_index} = obj.boats(boat_index).simulate( ...
                    obj.waypoints{boat_index}, obj.simulation_total_time_sec);
            end

            boat_true = cell(num_boats, 1);
            boat_est = cell(num_boats, 1);
            for boat_index = 1:num_boats
                boat_true{boat_index} = world_traj_to_radar_frame(sims{boat_index}.ground_truth_trajectory, obj.radar_pose_world);
                boat_est{boat_index} = world_traj_to_radar_frame(sims{boat_index}.est_trajectory, obj.radar_pose_world);
            end

            rng(obj.radar_rng_seed);
            measurements_actual = cell(num_boats, 1);
            measurements_prototype = cell(num_boats, 1);
            for boat_index = 1:num_boats
                measurements_actual{boat_index} = obj.true_radar_model.measure(boat_true{boat_index});
                measurements_prototype{boat_index} = obj.prototype_radar_model.measure(boat_est{boat_index});
            end

            poses_for_fit = boat_true;
            if obj.fit_using_estimated_poses
                poses_for_fit = boat_est;
            end

            fit_result = obj.solver.solve(poses_for_fit, measurements_actual);

            identified_model = ParametricRadarSensorModel( ...
                obj.solver.bias_model, obj.solver.variance_model, ...
                "bias_parameters", fit_result.bias_parameters, ...
                "variance_parameters", fit_result.variance_parameters, ...
                "enable_noise", false);

            results = struct();
            results.sims = sims;
            results.boat_true = boat_true;
            results.boat_est = boat_est;
            results.measurements_actual = measurements_actual;
            results.measurements_prototype = measurements_prototype;
            results.fit_result = fit_result;
            results.identified_model = identified_model;
            results.fit_using_estimated_poses = obj.fit_using_estimated_poses;
        end
    end
end
