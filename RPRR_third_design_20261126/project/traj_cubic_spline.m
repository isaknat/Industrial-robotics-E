function [q, qd, qdd] = traj_cubic_spline(tk, Qv, t)
% Cubic spline through a list of via points, starting and ending at rest
% (Lez_05 p.30-33). One cubic per segment; position, speed and acceleration
% are continuous at every via point in between.
%   tk   [1 x n] time of each via point (increasing)
%   Qv   [nj x n] via points, one row per joint
%   t    [1 x m] times at which the motion is evaluated
%   q, qd, qdd  [nj x m] position, speed, acceleration
%
% Written as on slide p.32, with the time counted from the start of each segment:
%   S_j(t) = a_j*(t - t_j)^3 + b_j*(t - t_j)^2 + c_j*(t - t_j) + theta_j
% Unknowns a_j, b_j, c_j for the n-1 segments; equations:
%   each segment ends on the next via point          (n-1)
%   speed and acceleration continuous at inner points (2*(n-2))
%   zero speed at the start and at the end            (2)

[nj, n] = size(Qv);
ns  = n - 1;
tau = diff(tk);
M   = zeros(3*ns);  rhs = zeros(3*ns, nj);
ia = @(j) 3*(j-1) + 1;  ib = @(j) 3*(j-1) + 2;  ic = @(j) 3*(j-1) + 3;
row = 0;
for j = 1:ns                                  % segment j ends on via point j+1
    row = row + 1;
    M(row, [ia(j) ib(j) ic(j)]) = [tau(j)^3, tau(j)^2, tau(j)];
    rhs(row,:) = (Qv(:,j+1) - Qv(:,j)).';
end
for j = 1:ns-1                                % continuity at via point j+1
    row = row + 1;                            % speed
    M(row, [ia(j) ib(j) ic(j)]) = [3*tau(j)^2, 2*tau(j), 1];
    M(row, ic(j+1)) = -1;
    row = row + 1;                            % acceleration
    M(row, [ia(j) ib(j)]) = [6*tau(j), 2];
    M(row, ib(j+1)) = -2;
end
row = row + 1;  M(row, ic(1)) = 1;                                        % start at rest
row = row + 1;  M(row, [ia(ns) ib(ns) ic(ns)]) = [3*tau(ns)^2, 2*tau(ns), 1];   % end at rest
X = M\rhs;                                    % [3*ns x nj]

seg = discretize(t, tk);                      % segment of each time sample
seg(t <= tk(1)) = 1;  seg(t >= tk(end)) = ns;
s = t - tk(seg);
q = zeros(nj, numel(t));  qd = q;  qdd = q;
for i = 1:nj
    a = X(ia(seg), i).';  b = X(ib(seg), i).';  c = X(ic(seg), i).';
    q(i,:)   = a.*s.^3 + b.*s.^2 + c.*s + Qv(i, seg);
    qd(i,:)  = 3*a.*s.^2 + 2*b.*s + c;
    qdd(i,:) = 6*a.*s + 2*b;
end
end
