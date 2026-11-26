function [Fq, Fin, Fgr, Mbar] = RPRRdyn(Q, Qp, Qpp, L, D, Ftip)
% Inverse dynamics of the R-P-R-R robot: the joint torques and force needed
% for a given motion (Lez_03 p.5-7, handout "SCARA Dynamics"):
%     Fq = -Je' * ( -M*Se_ddot + Fse ),   Se_ddot = Jep*Qp + Je*Qpp
%   Q, Qp, Qpp  joint position, speed, acceleration in SI: [rad; m; rad; rad] and derivatives
%   L           dimensions with h, L3, L4 in metres
%   D           masses, see robot_masses
%   Ftip        optional external force on the pen tip [3x1, N]
%   Fq    [4x1] what the joints must deliver: [N m; N; N m; N m]
%   Fin   part of Fq that moves the masses: (Je'*M*Je)*Qpp + (Je'*M*Jep)*Qp
%   Fgr   part of Fq that holds the weight (and Ftip): -Je'*Fse
%   Mbar  [4x4] mass seen by the joints, Je'*M*Je

if nargin < 6, Ftip = [0; 0; 0]; end
np  = size(D.pts, 1);
Je  = RPRRjacdin(Q, L, D);
Jep = RPRRjacPdin(Q, Qp, L, D);
Mv  = [D.J1; kron(D.m(:), [1; 1; 1])];           % diagonal of M, same order as Se
Fse = zeros(1 + 3*np, 1);
Fse(4:3:end) = -D.m(:)*D.g;                      % weight of each point, along -z
Fse(end-2:end) = Fse(end-2:end) + Ftip;
Mbar = Je.'*(Mv.*Je);
Fin  = Mbar*Qpp + Je.'*(Mv.*(Jep*Qp));
Fgr  = -Je.'*Fse;
Fq   = Fin + Fgr;
end
