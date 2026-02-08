function print_theta_comparison(theta_true_equiv, theta_est, parameter_names)
%PRINT_THETA_COMPARISON Print a table comparing parameter vectors.

theta_true_equiv = theta_true_equiv(:);
theta_est = theta_est(:);

if numel(theta_true_equiv) ~= numel(theta_est)
    error("print_theta_comparison:SizeMismatch", "Theta vectors must be the same length.");
end

if nargin < 3 || isempty(parameter_names)
    parameter_names = compose("theta_%02d", 1:numel(theta_true_equiv)).';
end

fprintf("%-30s | %14s | %14s | %14s\n", "Parameter", "True", "Estimated", "Error");

for param_index = 1:numel(theta_true_equiv)
    true_value = theta_true_equiv(param_index);
    est_value = theta_est(param_index);
    err_value = est_value - true_value;
    fprintf("%-30s | %14.6g | %14.6g | %14.6g\n", parameter_names(param_index), true_value, est_value, err_value);
end
end
