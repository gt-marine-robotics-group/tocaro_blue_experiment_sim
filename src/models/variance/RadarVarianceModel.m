classdef (Abstract) RadarVarianceModel
    %RadarVarianceModel Base interface for range-dependent variance models.

    methods (Abstract)
        num_params = parameterCount(obj)
        [range_variance_m2, bearing_variance_rad2] = evaluate(obj, range_m, theta, options)
        [range_jacobian, bearing_jacobian] = jacobian(obj, range_m, theta, options)
        theta_initial = initialGuess(obj, range_m, range_residual_m, bearing_residual_rad, options)
        param_labels = parameterLabels(obj)
        [A, b] = inequalityConstraints(obj, range_m, options)
    end
end
