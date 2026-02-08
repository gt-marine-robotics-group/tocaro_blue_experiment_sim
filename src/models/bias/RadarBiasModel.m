classdef (Abstract) RadarBiasModel
    %RadarBiasModel Base interface for range-dependent bias models.

    methods (Abstract)
        num_params = parameterCount(obj)
        [range_bias_m, bearing_bias_rad] = evaluate(obj, range_m, theta)
        [range_jacobian, bearing_jacobian] = jacobian(obj, range_m, theta)
        theta_initial = initialGuess(obj, range_m, predicted_bearing_rad, measured_range_m, measured_bearing_rad)
        param_labels = parameterLabels(obj)
    end
end
