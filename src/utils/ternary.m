function out_value = ternary(condition_value, true_value, false_value)
%TERNARY Simple ternary helper.

if condition_value
    out_value = true_value;
else
    out_value = false_value;
end
end
