function ellipse_handle = plot_cov_ellipse_2d(x_center, y_center, position_covariance, sigma_multiplier)
%PLOT_COV_ELLIPSE_2D Plot a covariance ellipse in 2D.

ellipse_handle = [];

if isempty(position_covariance) || ~all(size(position_covariance) == [2 2]) || any(~isfinite(position_covariance), "all")
    return;
end

position_covariance = (position_covariance + position_covariance')/2;

[eig_vectors, eig_values] = eig(position_covariance);
eig_diagonal = diag(eig_values);
eig_diagonal(eig_diagonal < 0) = 0;

angle_samples = linspace(0, 2*pi, 80);
semi_axis_1 = sigma_multiplier * sqrt(eig_diagonal(1));
semi_axis_2 = sigma_multiplier * sqrt(eig_diagonal(2));

ellipse_points = eig_vectors * [semi_axis_1*cos(angle_samples); semi_axis_2*sin(angle_samples)];

ellipse_x = x_center + ellipse_points(1,:);
ellipse_y = y_center + ellipse_points(2,:);

ellipse_handle = plot(ellipse_x, ellipse_y, "-", "LineWidth", 1.0);
end
