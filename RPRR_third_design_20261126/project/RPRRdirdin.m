function Se = RPRRdirdin(Q, L, D)
% Extended direct kinematics for the dynamic model (like SCARAdirdin of the
% handout "SCARA Dynamics"): where every point mass is.
%   Q   [q1; q2; q3; q4] in [rad; m; rad; rad]   (SI: lengths in metres here)
%   L   dimensions with h, L3, L4 in metres
%   D   masses, see robot_masses
%   Se  extended vector [q1; x1; y1; z1; ...; x5; y5; z5]: the turn of the base
%       (it carries the moment of inertia J1), then the position of each point mass
%
% A point at fraction a3 of the q3 link and a4 of the tool is the "pen tip" of
% a shorter robot with L3 -> a3*L3 and L4 -> a4*L4, so RPRRdir can be reused.
% after CAD edition 1 (2026-11-24): a third column s of D.pts puts a point on the slider,
% s metres along the rail from the q3 joint: the same robot with q2 -> q2 + s.

np = size(D.pts, 1);
Se = zeros(1 + 3*np, 1);
Se(1) = Q(1);
for i = 1:np
    [Qi, Li] = point_robot(Q, L, D.pts(i,:));
    Se(3*i-1 : 3*i+1) = RPRRdir(Qi, Li);
end
end

function [Qi, Li] = point_robot(Q, L, p)
Li = L;  Li.L3 = p(1)*L.L3;  Li.L4 = p(2)*L.L4;
Qi = Q;  if numel(p) > 2, Qi(2) = Q(2) + p(3); end
end
