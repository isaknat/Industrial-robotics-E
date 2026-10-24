function [T, n, P, S] = RPRRdir(Q, L)
% Direct kinematics of the R-P-R-R robot: joint values -> pen tip and pen direction.
%   Q  [q1; q2; q3; q4] in [rad; mm; rad; rad]
%   L  dimensions, see robot_params (conventions in RPRRinv)
%   T  [3x1] pen tip position [mm]
%   n  [3x1] pen direction (unit)
%   P  [3x5] points for plotting: base, shoulder, q3 joint, q4 joint, pen tip
%   S  [4x1] workspace vector S = F(Q) = [x; y; z; theta], theta = pen angle above horizontal [rad]

e  = [-sin(Q(1)); cos(Q(1)); 0];           % horizontal direction of the arm plane
ez = [0; 0; 1];
th3 = L.alpha + Q(3);                      % absolute angle of the q3 link
th4 = th3 + Q(4);                          % absolute angle of the pen

B  = [0; 0; 0];
Sh = [0; 0; L.h];                          % shoulder: start of the rail
J  = Sh + Q(2)*(cos(L.alpha)*e + sin(L.alpha)*ez);
W  = J + L.L3*(cos(th3)*e + sin(th3)*ez);
n  = cos(th4)*e + sin(th4)*ez;
T  = W + L.L4*n;
P  = [B, Sh, J, W, T];
S  = [T; th4];
end
