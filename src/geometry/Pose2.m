classdef Pose2
    % Representation of 2D pose using the SE(2) Special Euclidean group manifold

    properties
        x     (1,1) double {mustBeNumeric} = 0.0
        y     (1,1) double {mustBeNumeric} = 0.0
        theta (1,1) double {mustBeNumeric} = 0.0

        timestamp (1,1) datetime = NaT
        covariance (3,3) double = eye(3)
    end

    methods
        function obj = Pose2(x, y, theta, varargin)
            % Pose2() OR Pose2(x,y,theta, Name,Value,...)
            %
            % Name-Value options:
            %   "timestamp"  : datetime scalar (default NaT)
            %   "covariance" : 3x3 double (default eye(3))

            if nargin == 0
                return;
            end

            if nargin < 3
                error("Pose2 constructor expects Pose2() or Pose2(x,y,theta,...).");
            end

            obj.x = x;
            obj.y = y;
            obj.theta = wrap_to_pi(theta);

            % ---- Parse optional args ----
            if ~isempty(varargin)
                p = inputParser;
                p.FunctionName = "Pose2";

                addParameter(p, "timestamp", NaT, @(t) isdatetime(t) && isscalar(t));
                addParameter(p, "covariance", eye(3), @(C) isnumeric(C) && isequal(size(C), [3 3]));

                parse(p, varargin{:});

                obj.timestamp = p.Results.timestamp;
                obj.covariance = Pose2.validate_covariance(p.Results.covariance);
            else
                % Ensure defaults are valid
                obj.covariance = Pose2.validate_covariance(obj.covariance);
            end
        end

        function R = rot(obj)
            % Return the rotation component as a Rot2 object
            R = Rot2(obj.theta);
        end

        function T = matrix(obj)
            % Homogeneous transform (3x3) in SE(2)
            R = obj.rot().matrix();
            t = [obj.x; obj.y];

            T = [R, t;
                 0, 0, 1];
        end

        function p_out = transform(obj, p_in)
            % Apply the pose to a 2D point or set of points.
            % Accepts 2xN or Nx2 and returns same shape.

            wasRowFormat = false;

            if size(p_in,1) == 2
                P = p_in;           % 2xN
            elseif size(p_in,2) == 2
                P = p_in.';         % 2xN
                wasRowFormat = true;
            else
                error("transform expects points as 2xN or Nx2.");
            end

            R = obj.rot().matrix();
            t = [obj.x; obj.y];

            P_out = R * P + t;

            if wasRowFormat
                p_out = P_out.';    % Nx2
            else
                p_out = P_out;      % 2xN
            end
        end

        function T3 = mtimes(T1, T2)
            % Group composition: T3 = T1 * T2
            if ~isa(T1, "Pose2") || ~isa(T2, "Pose2")
                error("Pose2 multiplication is only defined for Pose2 * Pose2.");
            end

            R1 = T1.rot().matrix();
            t1 = [T1.x; T1.y];

            R2 = T2.rot().matrix();
            t2 = [T2.x; T2.y];

            R3 = R1 * R2;
            t3 = R1 * t2 + t1;

            theta3 = atan2(R3(2,1), R3(1,1));
            T3 = Pose2(t3(1), t3(2), theta3);

            % TODO: figure out best way to set timestamp and cov data from compose
            T3.timestamp  = NaT;
            T3.covariance = eye(3);
        end

        function Tinv = inv(T)
            % Group inverse: Tinv = T^{-1}

            R = T.rot().matrix();
            t = [T.x; T.y];

            Rinv = R.';
            tinv = -Rinv * t;

            theta_inv = wrap_to_pi(-T.theta);
            Tinv = Pose2(tinv(1), tinv(2), theta_inv);

            Tinv.timestamp  = T.timestamp;
            Tinv.covariance = T.covariance;
        end
    end

    methods (Static, Access = private)
        function C = validate_covariance(C)
            if any(~isfinite(C), "all")
                error("Pose2.covariance must contain finite values.");
            end
            if norm(C - C.', 'fro') > 1e-10
                error("Pose2.covariance must be symmetric.");
            end

            C = (C + C.')/2;
        end
    end
end
