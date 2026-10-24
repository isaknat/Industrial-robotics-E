function [k_inv, sv] = RPRRscore(Q, L)
% Kinematic quality of the R-P-R-R robot at one pose (Project requirement 5):
% inverse condition number of the Jacobian, 1/kappa = sigma_min/sigma_max
% (Lez_04 p.23: 1 = isotropic, 0 = singular, above 0.2 acceptable).
%
% J mixes units (q2 in mm, the other joints in rad; theta row in rad), so it is
% first made dimensionless with the characteristic length L.Lc:
%   - a rotary joint turning by dq counts like a slider moving by Lc*dq
%   - a pen tilt d(theta) counts like a pen-tip motion of Lc*d(theta)
% The choice of Lc is a PLACEHOLDER (see robot_params); once motors and
% transmissions are chosen, normalizing by the joint speed limits is the
% alternative suggested in Lez_04 p.24.

J  = RPRRjac(Q, L);
Jn = diag([1 1 1 L.Lc]) * J * diag([1/L.Lc, 1, 1/L.Lc, 1/L.Lc]);
sv = svd(Jn);
k_inv = sv(end)/sv(1);
end
