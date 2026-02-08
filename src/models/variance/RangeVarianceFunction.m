classdef (Abstract) RangeVarianceFunction
    %RangeVarianceFunction Base interface for single-channel variance models.

    methods (Abstract)
        num_params = parameterCount(obj)
        variance = evaluate(obj, range_m, theta, options)
        jacobian = jacobian(obj, range_m, theta, options)
        theta_initial = initialGuess(obj, range_m, residual, options)
        param_labels = parameterLabels(obj, unit_label)
        [A, b] = inequalityConstraints(obj, range_m, options)
    end
end
