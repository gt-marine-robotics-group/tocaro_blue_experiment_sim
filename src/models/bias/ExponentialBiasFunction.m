classdef ExponentialBiasFunction < RangeBiasFunction
    %ExponentialBiasFunction Exponential bias function for a single channel.

    methods
        function num_params = parameterCount(~)
            num_params = 2;
        end

        function bias = evaluate(~, range_m, theta)
            theta = theta(:);
            bias = theta(1) .* exp(theta(2) .* range_m);
        end

        function jacobian = jacobian(~, range_m, theta)
            theta = theta(:);
            exp_term = exp(theta(2) .* range_m);
            jacobian = zeros(numel(range_m), 2);
            jacobian(:, 1) = exp_term;
            jacobian(:, 2) = theta(1) .* exp_term .* range_m;
        end

        function theta_initial = initialGuess(~, range_m, residual)
            residual = residual(:);
            range_m = range_m(:);
            same_sign = all(residual ~= 0) && all(sign(residual) == sign(residual(1)));

            if same_sign
                log_residual = log(abs(residual));
                design_matrix = [ones(numel(range_m), 1), range_m];
                beta = design_matrix \log_residual;
                theta_initial = [sign(mean(residual)) * exp(beta(1)); beta(2)];
            else
                theta_initial = [mean(residual); 0.0];
            end
        end

        function param_labels = parameterLabels(~, unit_label)
            param_labels = [
                "scale_" + unit_label
                "rate_per_m"
            ];
        end
    end
end
