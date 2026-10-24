function [Q, ok] = RPRRinv(T, n, L)
% Inverse kinematics of the R-P-R-R robot: pen tip position and pen direction -> joint values.
%   T  [3x1] pen tip position [mm]
%   n  [3x1] pen direction (unit vector, from the robot into the wall)
%   L  dimensions, see robot_params
%   Q  [q1; q2; q3; q4] in [rad; mm; rad; rad], NaN if out of reach
%   ok false if the point cannot be reached
%
% Conventions (in the arm's vertical plane after q1; rho = horizontal distance from the axis, z up):
%   q1  rotation about z; q1 = 0 -> arm plane along +y (as drawn in PDF p.2), positive counterclockwise seen from above
%   q2  slider position along the rail, measured from the axis at shoulder height
%   q3  angle of the q3 link relative to the rail, positive = tilts up
%   q4  angle of the pen relative to the q3 link, positive = tilts up
%
% Steps (block 2a): pen direction is known, so step back one pen length from the tip
% to get the q4 joint W; the q3 joint lies on the rail at distance L3 from W
% (circle around W meets the rail line); read the angles from the points.

% q1: turn the arm plane towards the target
q1 = atan2(-T(1), T(2));
e  = [-sin(q1); cos(q1); 0];               % horizontal direction of the arm plane

% Everything else happens in the arm plane: (rho, z)
Trz = [e.'*T;  T(3)];
nrz = [e.'*n;  n(3)];
W   = Trz - L.L4*nrz;                      % q4 joint

% q3 joint J = S + q2*u on the rail, with |J - W| = L3
S = [0; L.h];
u = [cos(L.alpha); sin(L.alpha)];
D = W - S;
b = u.'*D;
disc = b^2 - D.'*D + L.L3^2;
ok = disc >= 0;
if ~ok
    Q = nan(4, 1);
    return
end
q2 = b + L.sol*sqrt(disc);
J  = S + q2*u;

th3 = atan2(W(2) - J(2), W(1) - J(1));     % absolute angle of the q3 link
th4 = atan2(nrz(2), nrz(1));               % absolute angle of the pen
q3  = wrap(th3 - L.alpha);
q4  = wrap(th4 - th3);
Q   = [q1; q2; q3; q4];
end

function a = wrap(a)
a = atan2(sin(a), cos(a));
end
