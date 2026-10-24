function [Q, Qd, Qdd] = traj_spline(tv, Qv, tt)
% Cubic spline in joint space through via points, at rest at both ends (Lez_05).
%   tv   [1xn] times of the via points [s], starting at 0
%   Qv   [m x n] joint values at the via points (one row per joint)
%   tt   times to evaluate
%   Q, Qd, Qdd  [m x numel(tt)] position, velocity, acceleration
%
% Adding a 0 before and after the values makes MATLAB's spline "clamped":
% velocity 0 at the first and last via point. Acceleration there is NOT 0.
% Between via points a spline can overshoot, so check the limits on the whole curve.

m  = size(Qv, 1);
pp = spline(tv, [zeros(m, 1), Qv, zeros(m, 1)]);

% Differentiate the polynomial pieces (no toolbox needed)
[br, c, ~, k, d] = unmkpp(pp);
ppd  = mkpp(br, c(:, 1:k-1) .* (k-1:-1:1), d);
ppdd = mkpp(br, c(:, 1:k-2) .* (k-2:-1:1) .* (k-1:-1:2), d);

Q   = ppval(pp,   tt);
Qd  = ppval(ppd,  tt);
Qdd = ppval(ppdd, tt);
end
