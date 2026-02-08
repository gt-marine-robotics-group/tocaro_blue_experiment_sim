classdef Radar < handle
    properties
        true_trajectories (:,1) Pose2 = Pose2.empty
        est_trajectories  (:,1) Pose2 = Pose2.empty
        actual_measurements (:,1) RadarMeasurement = RadarMeasurement.empty

        true_sensor_model RadarSensorModel = SimpleSensorModel.empty
    end

    methods
        function obj = Radar(true_sensor_model)
            if nargin == 1
                obj.true_sensor_model = true_sensor_model;
            end
        end

        function sim_radar(obj, true_trajectories, est_trajectories)
            if isempty(true_trajectories) || isempty(est_trajectories)
                error("Radar:EmptyTrajectories", "true_trajectories and est_trajectories must be non-empty.");
            end

            % ---- Sort true trajectory by timestamp ----
            true_timestamps = [true_trajectories.timestamp].';
            [~, true_sort_idx] = sort(true_timestamps, "ascend");
            obj.true_trajectories = true_trajectories(true_sort_idx);

            % ---- Sort estimated trajectory by timestamp ----
            est_timestamps = [est_trajectories.timestamp].';
            [~, est_sort_idx] = sort(est_timestamps, "ascend");
            obj.est_trajectories = est_trajectories(est_sort_idx);

            % ---- Generate actual measurements from ground truth ----
            obj.actual_measurements = obj.true_sensor_model.measure(obj.true_trajectories);
        end

        function [rmse_model_whitened, rmse_true_whitened] = test_prototype_model(obj, prototype_model)
            if isempty(obj.est_trajectories) || isempty(obj.actual_measurements)
                error("Radar:NotSimulated", "Must run sim_radar() before test_prototype_model().");
            end

            predicted_measurements = prototype_model.measure(obj.est_trajectories);

            residuals = predicted_measurements - obj.actual_measurements;

                model_whitened = RadarMeasurement.whitenResidualUsingSigmas(predicted_measurements, residuals);
            rmse_model_whitened = sqrt(mean(sum(model_whitened.^2, 2)));

            % --- Whiten using TRUE (evaluation only, simulation/offline) ---
            true_whitened = RadarMeasurement.whitenResidualUsingSigmas(obj.actual_measurements, residuals);
            rmse_true_whitened = sqrt(mean(sum(true_whitened.^2, 2)));
        end
    end
end
