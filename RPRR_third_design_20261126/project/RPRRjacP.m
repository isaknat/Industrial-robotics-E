function Jp = RPRRjacP(Q, Qp, L)
% Time derivative of the Jacobian of the R-P-R-R robot: S_ddot = Jp*Q_dot + J*Q_ddot.
%   Q   [q1; q2; q3; q4] in [rad; mm; rad; rad]
%   Qp  joint velocities [rad/s; mm/s; rad/s; rad/s]
% Same symbols as RPRRjac; w3, w4 are the angular speeds of the q3 link and of the pen.

s1 = sin(Q(1));  c1 = cos(Q(1));  w1 = Qp(1);
ca = cos(L.alpha);
th3 = L.alpha + Q(3);  th4 = th3 + Q(4);
w3 = Qp(3);  w4 = Qp(3) + Qp(4);

rho = Q(2)*ca + L.L3*cos(th3) + L.L4*cos(th4);
a = -L.L3*sin(th3) - L.L4*sin(th4);
b = -L.L4*sin(th4);

rhop = Qp(2)*ca - L.L3*sin(th3)*w3 - L.L4*sin(th4)*w4;
ap = -L.L3*cos(th3)*w3 - L.L4*cos(th4)*w4;
bp = -L.L4*cos(th4)*w4;
cp = -L.L3*sin(th3)*w3 - L.L4*sin(th4)*w4;
dp = -L.L4*sin(th4)*w4;

Jp = [ s1*w1*rho - c1*rhop,  -c1*w1*ca,  -c1*w1*a - s1*ap,  -c1*w1*b - s1*bp;
      -c1*w1*rho - s1*rhop,  -s1*w1*ca,  -s1*w1*a + c1*ap,  -s1*w1*b + c1*bp;
       0,                     0,          cp,                dp;
       0,                     0,          0,                 0 ];
end
