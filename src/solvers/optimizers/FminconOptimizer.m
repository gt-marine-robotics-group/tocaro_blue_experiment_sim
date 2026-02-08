classdef FminconOptimizer < OptimizerBase
    %FminconOptimizer Constrained optimization wrapper.

    properties
        algorithm (1,1) string = "interior-point"
    end

    methods
        function [theta_opt, fval, exitflag, output] = optimize(obj, objective_handle, theta_initial, options)
            if exist("fmincon", "file") ~= 2
                error("FminconOptimizer:Unavailable", "fmincon not found. Install Optimization Toolbox.");
            end

            opt = optimoptions("fmincon", ...
                "Algorithm", char(obj.algorithm), ...
                "Display", char(options.display), ...
                "MaxIterations", options.max_iterations, ...
                "SpecifyObjectiveGradient", options.use_analytic_gradient);

            A = [];
            b = [];
            if isfield(options, "A"); A = options.A; end
            if isfield(options, "b"); b = options.b; end

            [theta_opt, fval, exitflag, output] = ...
                fmincon(objective_handle, theta_initial, A, b, [], [], [], [], [], opt);
        end
    end
end
