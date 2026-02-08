classdef (Abstract) RangeBiasFunction
    %RangeBiasFunction Base interface for single-channel bias functions.

    methods (Abstract)
        num_params = parameterCount(obj)
        bias = evaluate(obj, range_m, theta)
        jacobian = jacobian(obj, range_m, theta)
        theta_initial = initialGuess(obj, range_m, residual)
        param_labels = parameterLabels(obj, unit_label)
    end
end
