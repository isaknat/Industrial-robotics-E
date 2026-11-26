function T = traj_min_time(dq, V, A)
% Minimum actuating time of a rest-to-rest move with a speed limit V and an
% acceleration limit A (same limit for braking). Lez_05 "Minimum time motion
% curve"; same formulas as min_rise_time.m of Ex3.1 with A = D.
%   dq  displacement(s), any sign
%   V   maximum speed, A maximum acceleration (same size as dq, or scalars)
%   T   minimum time for each displacement
%
% Short move (the speed limit is never reached): triangle, T = 2*sqrt(dq/A).
% Long move: trapezoid, T = dq/V + V/A.

dq = abs(dq);
V  = V + zeros(size(dq));
A  = A + zeros(size(dq));
T  = dq./V + V./A;
short = dq < V.^2./A;
T(short) = 2*sqrt(dq(short)./A(short));
end
