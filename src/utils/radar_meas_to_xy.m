function [xy_m, range_m, theta_rad] = radar_meas_to_xy(measurements)
%RADAR_MEAS_TO_XY Convert radar measurements to xy positions.

if isempty(measurements)
    xy_m = zeros(0,2);
    range_m = zeros(0,1);
    theta_rad = zeros(0,1);
    return;
end

range_m  = reshape([measurements.range], [], 1);
theta_rad = reshape([measurements.theta], [], 1);

xy_m = [range_m .* cos(theta_rad), range_m .* sin(theta_rad)];
end
