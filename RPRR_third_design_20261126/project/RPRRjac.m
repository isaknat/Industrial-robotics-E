function J = RPRRjac(Q, L)
% Jacobian matrix of the R-P-R-R robot: S_dot = J*Q_dot, with S = [x; y; z; theta] (see RPRRdir).
%   Q  [q1; q2; q3; q4] in [rad; mm; rad; rad]
%   J  [4x4]; columns 1,3,4 in mm/rad (theta row: rad/rad), column 2 in mm/mm
%
% In the arm plane the pen tip is at
%   rho = q2*cos(alpha) + L3*cos(th3) + L4*cos(th4)      (distance from the axis)
%   z   = h + q2*sin(alpha) + L3*sin(th3) + L4*sin(th4)
% with th3 = alpha + q3, th4 = alpha + q3 + q4, and x = -sin(q1)*rho, y = cos(q1)*rho.
% det(J) = -rho*L3*cos(q3): singular when q3 = +-90 deg (q3 link perpendicular to the rail)
% or when the pen tip is on the axis.

s1 = sin(Q(1));  c1 = cos(Q(1));
ca = cos(L.alpha);  sa = sin(L.alpha);
th3 = L.alpha + Q(3);  th4 = th3 + Q(4);

rho = Q(2)*ca + L.L3*cos(th3) + L.L4*cos(th4);
a = -L.L3*sin(th3) - L.L4*sin(th4);        % d rho / d q3
b = -L.L4*sin(th4);                        % d rho / d q4
c =  L.L3*cos(th3) + L.L4*cos(th4);        % d z / d q3
d =  L.L4*cos(th4);                        % d z / d q4

J = [ -c1*rho, -s1*ca, -s1*a, -s1*b;
      -s1*rho,  c1*ca,  c1*a,  c1*b;
       0,       sa,     c,     d;
       0,       0,      1,     1 ];
end
