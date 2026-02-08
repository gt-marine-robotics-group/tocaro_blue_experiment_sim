classdef FminsearchOptimizer < OptimizerBase
    %FminsearchOptimizer Nelder-Mead fallback optimizer.

    methods
        function [theta_opt, fval, exitflag, output] = optimize(~, objective_handle, theta_initial, options)
            opt = optimset("Display", char(options.display), "MaxIter", options.max_iterations);
            [theta_opt, fval, exitflag, output] = fminsearch(objective_handle, theta_initial, opt);
        end
    end
end
