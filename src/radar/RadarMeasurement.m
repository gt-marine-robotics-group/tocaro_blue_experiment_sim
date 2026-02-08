classdef RadarMeasurement
    properties
        theta (1,1) double {mustBeNumeric} = 0.0
        range (1,1) double {mustBeNumeric, mustBePositive} = 1.0

        theta_sigma (1,1) double {mustBeNumeric, mustBePositive} = 1e-3
        theta_bias  (1,1) double {mustBeNumeric} = 0.0

        range_sigma (1,1) double {mustBeNumeric, mustBePositive} = 0.1
        range_bias  (1,1) double {mustBeNumeric} = 0.0

        target_id (1,1) string {mustBeTextScalar} = ""
    end

    methods
        function obj = RadarMeasurement(theta, range, target_id)
            if nargin == 3
                obj.theta = theta;
                obj.range = range;
                obj.target_id = target_id;
            elseif nargin ~= 0
                error("RadarMeasurement expects RadarMeasurement() or RadarMeasurement(theta, range, target_id).");
            end
        end

        function pose = getPose(obj, varargin)
            % getPose Convert (theta, range) into Pose2 with covariance.

            parser = inputParser;
            parser.addParameter("apply_bias", true, @(v) islogical(v) && isscalar(v));
            parser.addParameter("use_heading", true, @(v) islogical(v) && isscalar(v)); % if false, theta=0 and no heading covariance
            parser.parse(varargin{:});
            apply_bias  = parser.Results.apply_bias;
            use_heading = parser.Results.use_heading;

            measured_theta = obj.theta;
            measured_range = obj.range;

            if apply_bias
                measured_theta = measured_theta + obj.theta_bias;
                measured_range = measured_range + obj.range_bias;
            end

            % Polar -> Cartesian in radar frame
            x_pos = measured_range * cos(measured_theta);
            y_pos = measured_range * sin(measured_theta);

            heading = 0.0;
            if use_heading
                heading = measured_theta;
            end

            pose = Pose2(x_pos, y_pos, heading);

            % ---- Covariance propagation ----
            sigma_r     = obj.range_sigma;
            sigma_theta = obj.theta_sigma;

            R_polar = diag([sigma_r^2, sigma_theta^2]); % assumes independent range/theta noise

            if use_heading
                J = [ cos(measured_theta), -measured_range*sin(measured_theta);
                    sin(measured_theta),  measured_range*cos(measured_theta);
                    0,                    1 ];
            else
                % If pose.theta is fixed to 0, heading does not depend on measurement
                J = [ cos(measured_theta), -measured_range*sin(measured_theta);
                    sin(measured_theta),  measured_range*cos(measured_theta);
                    0,                    0 ];
            end

            pose.covariance = J * R_polar * J.';
        end


        % ------ Overrides ------
        function residual = minus(meas_a, meas_b)
            % Returns numeric residuals: [delta_theta, delta_range]

            num_a = numel(meas_a);
            num_b = numel(meas_b);

            if num_a == 1 && num_b > 1
                meas_a = repmat(meas_a, size(meas_b));
            elseif num_b == 1 && num_a > 1
                meas_b = repmat(meas_b, size(meas_a));
            elseif ~isequal(size(meas_a), size(meas_b))
                error("RadarMeasurement.minus:SizeMismatch; Sizes must match or one operand must be scalar.");
            end

            theta_a = reshape([meas_a.theta], [], 1);
            theta_b = reshape([meas_b.theta], [], 1);

            range_a = reshape([meas_a.range], [], 1);
            range_b = reshape([meas_b.range], [], 1);

            delta_theta = wrap_to_pi(theta_a - theta_b);
            delta_range = range_a - range_b;

            residual = [delta_theta, delta_range];
        end

        function whitened_residual = whitenResidual(obj, residual)
            whitened_residual = RadarMeasurement.whitenResidualUsingSigmas(obj, residual);
        end
    end

    methods (Static)
        function whitened_residual = whitenResidualUsingSigmas(sigma_source_meas, residual)
            % whitenResidualUsingSigmas Whiten residuals using sigmas from sigma_source_meas.
            % sigma_source_meas: Nx1 RadarMeasurement (or scalar) providing theta_sigma/range_sigma
            % residual: Nx2 [d_theta, d_range]

            num_sigma = numel(sigma_source_meas);
            num_resid = size(residual, 1);

            if num_sigma == 1 && num_resid > 1
                sigma_source_meas = repmat(sigma_source_meas, num_resid, 1);
            elseif num_sigma ~= num_resid
                error("RadarMeasurement:WhitenSizeMismatch", ...
                    "sigma_source_meas must be scalar or have the same number of rows as residual.");
            end

            theta_sigmas = reshape([sigma_source_meas.theta_sigma], [], 1);
            range_sigmas = reshape([sigma_source_meas.range_sigma], [], 1);

            if any(theta_sigmas <= 0) || any(range_sigmas <= 0)
                error("RadarMeasurement:InvalidSigmas", "All sigmas must be positive for whitening.");
            end

            whitened_residual = residual;
            whitened_residual(:,1) = whitened_residual(:,1) ./ theta_sigmas;
            whitened_residual(:,2) = whitened_residual(:,2) ./ range_sigmas;
        end
    end
end
