classdef (Abstract) OptimizerBase
    %OptimizerBase Abstract optimizer interface.

    methods (Abstract)
        [theta_opt, fval, exitflag, output] = optimize(obj, objective_handle, theta_initial, options)
    end
end
