classdef FminuncOptimizer < OptimizerBase
    %FminuncOptimizer Quasi-Newton optimizer wrapper.

    properties
        algorithm (1,1) string = "quasi-newton"
    end

    methods
        function [theta_opt, fval, exitflag, output] = optimize(obj, objective_handle, theta_initial, options)
            if exist("fminunc", "file") ~= 2
                error("FminuncOptimizer:Unavailable", "fminunc not found. Install Optimization Toolbox.");
            end

            opt = optimoptions("fminunc", ...
                "Algorithm", char(obj.algorithm), ...
                "Display", char(options.display), ...
                "MaxIterations", options.max_iterations, ...
                "SpecifyObjectiveGradient", options.use_analytic_gradient);

            [theta_opt, fval, exitflag, output] = fminunc(objective_handle, theta_initial, opt);
        end
    end
end
