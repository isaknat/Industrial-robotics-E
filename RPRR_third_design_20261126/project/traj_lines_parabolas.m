function [q, qd, qdd, info] = traj_lines_parabolas(Qv, tau, A, t)
% "Lines and parabolas" through a list of via points (Lez_05 p.25-29, Ex3.2).
% Each joint moves at constant speed between via points and changes speed with
% a constant acceleration A around each via point. The motion starts and ends
% at rest exactly on the first and last via point (auxiliary points of the
% slides); the via points in between are passed nearby, not exactly.
%   Qv   [nj x n] via points, one row per joint
%   tau  [1 x n-1] time between consecutive via points (same for all joints)
%   A    [nj x 1] acceleration used in the parabolas, one per joint (> 0)
%   t    [1 x m] times at which the motion is evaluated, 0 <= t <= sum(tau)
%   q, qd, qdd  [nj x m] position, speed, acceleration
%   info.ok       false if the given times are too short for acceleration A
%   info.t_blend  [nj x n] duration of the parabola at each via point
%   info.dt_line  [nj x n-1] duration of each constant-speed part
%
% Formulas of the slides, for one joint (theta = via values):
%   line speed          v_j = (theta_j+1 - theta_j)/tau_j
%   parabola duration   t_j = |v_j - v_j-1|/A, half before and half after the via point
%   first segment       t_1 = tau_1 - sqrt(tau_1^2 - 2|theta_2 - theta_1|/A),  v_1 = (theta_2 - theta_1)/(tau_1 - t_1/2)
%   last segment        t_n = tau_n-1 - sqrt(tau_n-1^2 - 2|theta_n - theta_n-1|/A),  v_n-1 likewise
% With only two points the move is a plain trapezoid: t_1 = t_2 = (tau - sqrt(tau^2 - 4|d|/A))/2.

[nj, n] = size(Qv);
Tn = [0, cumsum(tau)];                       % nominal time of each via point
q = zeros(nj, numel(t));  qd = q;  qdd = q;
info.ok = true;  info.t_blend = zeros(nj, n);  info.dt_line = zeros(nj, n - 1);

for i = 1:nj
    th = Qv(i,:);  a = A(i);  d = diff(th);
    v  = d./tau;                             % speed of each line
    tb = zeros(1, n);                        % duration of the parabola at each via point
    if n == 2
        disc = tau^2 - 4*abs(d)/a;
        if disc < 0, info.ok = false; disc = 0; end
        tb(:) = (tau - sqrt(disc))/2;
        v = sign(d)*a*tb(1);
    else
        disc1 = tau(1)^2   - 2*abs(d(1))/a;
        discn = tau(end)^2 - 2*abs(d(end))/a;
        if disc1 < 0 || discn < 0, info.ok = false; end
        tb(1) = tau(1)   - sqrt(max(disc1, 0));
        tb(n) = tau(end) - sqrt(max(discn, 0));
        v(1)   = d(1)/(tau(1) - tb(1)/2);
        v(end) = d(end)/(tau(end) - tb(n)/2);
        tb(2:n-1) = abs(diff(v))/a;
    end

    % The acceleration is constant during each parabola and zero elsewhere
    ts  = [0, Tn(2:n-1) - tb(2:n-1)/2, Tn(n) - tb(n)];   % start of each parabola
    te  = ts + tb;                                        % end of each parabola
    dv  = diff([0, v, 0]);                                % speed change at each via point
    acc = zeros(1, n);  nz = tb > 0;
    acc(nz) = dv(nz)./tb(nz);
    dtl = ts(2:end) - te(1:end-1);                        % constant-speed parts
    if any(dtl < -1e-9), info.ok = false; end
    info.t_blend(i,:) = tb;  info.dt_line(i,:) = dtl;

    % Integrate the piecewise-constant acceleration from rest at theta_1
    qi = th(1) + zeros(size(t));  qdi = zeros(size(t));  qddi = zeros(size(t));
    for k = find(nz)
        c    = min(max(t - ts(k), 0), tb(k));             % time spent inside parabola k
        qdi  = qdi + acc(k)*c;
        qi   = qi  + acc(k)*(0.5*c.^2 + c.*max(t - te(k), 0));
        qddi = qddi + acc(k)*(t >= ts(k) & t < te(k));
    end
    q(i,:) = qi;  qd(i,:) = qdi;  qdd(i,:) = qddi;
end
end
