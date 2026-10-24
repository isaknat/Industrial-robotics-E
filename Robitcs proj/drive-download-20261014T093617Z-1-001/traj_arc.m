function P = traj_arc(P0, Pm, P1, s)
% Points on the circular arc that starts at P0, passes through Pm and ends at P1.
%   P0, Pm, P1  [3x1] points (not on one line)
%   s           [1xm] fractions of the arc, 0 = P0, 1 = P1
%   P           [3xm] points on the arc
%
% Centre of the circle through three points (a = P1 - P0, b = Pm - P0):
%   C = P0 + ((|a|^2 b - |b|^2 a) x (a x b)) / (2 |a x b|^2)
% then P(lambda) = C + R (cos(lambda) e1 + sin(lambda) e2) in the circle's plane.

a = P1 - P0;  b = Pm - P0;
axb = cross(a, b);
assert(norm(axb) > 1e-9*norm(a)*norm(b), 'traj_arc: the three points are on one line');
C  = P0 + cross(dot(a, a)*b - dot(b, b)*a, axb) / (2*dot(axb, axb));
R  = norm(P0 - C);
e1 = (P0 - C)/R;
e2 = cross(axb/norm(axb), e1);

ang = @(P, e2) mod(atan2(dot(P - C, e2), dot(P - C, e1)), 2*pi);
if ang(Pm, e2) > ang(P1, e2)               % go the other way round, so the arc passes Pm
    e2 = -e2;
end
lam = s*ang(P1, e2);
P = C + R*(e1*cos(lam) + e2*sin(lam));
end
