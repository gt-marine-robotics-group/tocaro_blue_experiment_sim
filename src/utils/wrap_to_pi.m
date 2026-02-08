function wrapped_angle = wrap_to_pi(angle_rad)
%WRAP_TO_PI Wrap angle(s) to [-pi, pi].
% Toolbox-free replacement for wrapToPi.

wrapped_angle = atan2(sin(angle_rad), cos(angle_rad));
end
