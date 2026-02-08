classdef Rot2
    properties
        theta (1,1) double {mustBeNumeric} = 0.0
    end

    methods
        function obj = Rot2(theta)
            if nargin == 1
                obj.theta = wrap_to_pi(theta);
            end
        end

        function R = matrix(obj)
            c = cos(obj.theta);
            s = sin(obj.theta);
            R = [c, -s;
                 s,  c];
        end

        function R3 = mtimes(R1, R2)
            R3 = Rot2(R1.theta + R2.theta);
        end

        function Rinv = inv(R)
            Rinv = Rot2(-R.theta);
        end

        function v_out = rotate(R, v)
            v_out = R.matrix() * v;
        end
    end
end
