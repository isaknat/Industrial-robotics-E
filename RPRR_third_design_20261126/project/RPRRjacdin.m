function Je = RPRRjacdin(Q, L, D)
% Extended Jacobian for the dynamic model: Se_dot = Je*Q_dot (like SCARAjacdin).
% [1+3*np x 4]. Units as in RPRRdirdin (SI). Built from RPRRjac, see RPRRdirdin.
% after CAD edition 1 (2026-11-24): third column of D.pts = offset along the rail (q2 -> q2 + s).

np = size(D.pts, 1);
Je = zeros(1 + 3*np, 4);
Je(1,:) = [1 0 0 0];
for i = 1:np
    Li = L;  Li.L3 = D.pts(i,1)*L.L3;  Li.L4 = D.pts(i,2)*L.L4;
    Qi = Q;  if size(D.pts, 2) > 2, Qi(2) = Q(2) + D.pts(i,3); end
    J = RPRRjac(Qi, Li);
    Je(3*i-1 : 3*i+1, :) = J(1:3,:);
end
end
