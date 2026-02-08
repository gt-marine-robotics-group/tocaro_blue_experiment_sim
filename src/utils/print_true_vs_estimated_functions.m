function print_true_vs_estimated_functions(true_model, fit_result)
%PRINT_TRUE_VS_ESTIMATED_FUNCTIONS Print bias/sigma forms for true vs estimated.

if ~isa(true_model, "ParametricRadarSensorModel")
    error("print_true_vs_estimated_functions:BadInput", ...
        "Expected true_model to be a ParametricRadarSensorModel.");
end

true_bias_params = true_model.bias_parameters(:);
true_var_params = true_model.variance_parameters(:);

fit_bias_params = fit_result.bias_parameters(:);
fit_var_params = fit_result.variance_parameters(:);

fmt_lin = @(a0,a1) sprintf("%.6g + %.6g*r", a0, a1);
fmt_quad = @(a0,a1,a2) sprintf("%.6g + %.6g*r + %.6g*r^2", a0, a1, a2);
fmt_exp = @(a0,a1) sprintf("%.6g*exp(%.6g*r)", a0, a1);

true_bias_str = format_bias_strings(true_model.bias_model, true_bias_params, fmt_lin, fmt_quad, fmt_exp);
fit_bias_str = format_bias_strings(true_model.bias_model, fit_bias_params, fmt_lin, fmt_quad, fmt_exp);

true_var_str = format_variance_strings(true_model.variance_model, true_var_params, fmt_lin, fmt_exp);
fit_var_str = format_variance_strings(true_model.variance_model, fit_var_params, fmt_lin, fmt_exp);

fprintf("%-20s | %-46s | %-46s\n", "Quantity", "True", "Estimated");
fprintf("%-20s | %-46s | %-46s\n", "Range bias b_r(r)", true_bias_str{1}, fit_bias_str{1});
fprintf("%-20s | %-46s | %-46s\n", "Bearing bias b_p(r)", true_bias_str{2}, fit_bias_str{2});
fprintf("%-20s | %-46s | %-46s\n", "Sigma/Var r(r)", true_var_str{1}, fit_var_str{1});
fprintf("%-20s | %-46s | %-46s\n", "Sigma/Var p(r)", true_var_str{2}, fit_var_str{2});
end

function bias_strings = format_bias_strings(bias_model, params, fmt_lin, fmt_quad, fmt_exp)
if isa(bias_model, "LinearBiasModel")
    bias_strings = {
        fmt_lin(params(1), params(2))
        fmt_lin(params(3), params(4))
    };
elseif isa(bias_model, "QuadraticBiasModel")
    bias_strings = {
        fmt_quad(params(1), params(2), params(3))
        fmt_quad(params(4), params(5), params(6))
    };
elseif isa(bias_model, "ExponentialBiasModel")
    bias_strings = {
        fmt_exp(params(1), params(2))
        fmt_exp(params(3), params(4))
    };
elseif isa(bias_model, "MixedBiasModel")
    bias_strings = format_mixed_bias_strings(bias_model, params, fmt_lin, fmt_quad, fmt_exp);
else
    bias_strings = {"<custom>", "<custom>"};
end
end

function variance_strings = format_variance_strings(variance_model, params, fmt_lin, fmt_exp)
if isa(variance_model, "LinearSigmaVarianceModel")
    variance_strings = {
        fmt_lin(params(1), params(2))
        fmt_lin(params(3), params(4))
    };
elseif isa(variance_model, "LogLinearVarianceModel")
    variance_strings = {
        fmt_lin(params(1), params(2))
        fmt_lin(params(3), params(4))
    };
elseif isa(variance_model, "ExponentialVarianceModel")
    variance_strings = {
        fmt_exp(params(1), params(2))
        fmt_exp(params(3), params(4))
    };
elseif isa(variance_model, "MixedVarianceModel")
    variance_strings = format_mixed_variance_strings(variance_model, params, fmt_lin, fmt_exp);
else
    variance_strings = {"<custom>", "<custom>"};
end
end

function bias_strings = format_mixed_bias_strings(bias_model, params, fmt_lin, fmt_quad, fmt_exp)
range_count = bias_model.range_model.parameterCount();
range_params = params(1:range_count);
bearing_params = params(range_count + 1:end);

bias_strings = {
    format_bias_function(bias_model.range_model, range_params, fmt_lin, fmt_quad, fmt_exp)
    format_bias_function(bias_model.bearing_model, bearing_params, fmt_lin, fmt_quad, fmt_exp)
};
end

function bias_str = format_bias_function(model, params, fmt_lin, fmt_quad, fmt_exp)
if isa(model, "LinearBiasFunction")
    bias_str = fmt_lin(params(1), params(2));
elseif isa(model, "QuadraticBiasFunction")
    bias_str = fmt_quad(params(1), params(2), params(3));
elseif isa(model, "ExponentialBiasFunction")
    bias_str = fmt_exp(params(1), params(2));
else
    bias_str = "<custom>";
end
end

function variance_strings = format_mixed_variance_strings(variance_model, params, fmt_lin, fmt_exp)
range_count = variance_model.range_model.parameterCount();
range_params = params(1:range_count);
bearing_params = params(range_count + 1:end);

variance_strings = {
    format_variance_function(variance_model.range_model, range_params, fmt_lin, fmt_exp)
    format_variance_function(variance_model.bearing_model, bearing_params, fmt_lin, fmt_exp)
};
end

function variance_str = format_variance_function(model, params, fmt_lin, fmt_exp)
if isa(model, "LinearSigmaVarianceFunction")
    variance_str = fmt_lin(params(1), params(2));
elseif isa(model, "LogLinearVarianceFunction")
    variance_str = fmt_lin(params(1), params(2));
elseif isa(model, "ExponentialVarianceFunction")
    variance_str = fmt_exp(params(1), params(2));
else
    variance_str = "<custom>";
end
end
