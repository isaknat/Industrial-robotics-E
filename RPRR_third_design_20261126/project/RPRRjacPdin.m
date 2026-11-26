function Jep = RPRRjacPdin(Q, Qp, L, D)
% Time derivative of the extended Jacobian: Se_ddot = Jep*Q_dot + Je*Q_ddot
% (like SCARAjacPdin). Units as in RPRRdirdin (SI). Built from RPRRjacP.
% after CAD edition 1 (2026-11-24): third column of D.pts = offset along the rail (q2 -> q2 + s).

np = size(D.pts, 1);
Jep = zeros(1 + 3*np, 4);
for i = 1:np
    Li = L;  Li.L3 = D.pts(i,1)*L.L3;  Li.L4 = D.pts(i,2)*L.L4;
    Qi = Q;  if size(D.pts, 2) > 2, Qi(2) = Q(2) + D.pts(i,3); end
    Jp = RPRRjacP(Qi, Qp, Li);
    Jep(3*i-1 : 3*i+1, :) = Jp(1:3,:);
end
end
