function [q, qd, qdd, ok] = traj_lfpb(qv, td, acc, tt)
% Lines and parabolas through via points, for ONE joint (Lez_05, Craig ch. 7).
%   qv   [1xn] joint values at the via points
%   td   [1x(n-1)] time from via point j to j+1 [s] (the same for all joints)
%   acc  blend acceleration |qdd| (positive), e.g. the joint's limit
%   tt   times to evaluate, from 0 (the first via point) to sum(td)
%   q, qd, qdd  position, velocity, acceleration at tt
%   ok   false if acc is too small for these times (a parabola would not fit)
%
% Starts and stops at rest and hits the first and last via point exactly.
% Constant velocity between via points, constant acceleration +-acc near them;
% the inner parabolas cut the corner and do NOT pass through the via point.

n  = numel(qv);
tb = zeros(1, n);                          % parabola (blend) duration at each via point
a  = zeros(1, n);                          % acceleration in each parabola
v  = zeros(1, n-1);                        % velocity on each line
ok = true;

if n == 2
    % One segment: trapezoidal velocity (accelerate, cruise, brake)
    dq   = qv(2) - qv(1);
    disc = td^2/4 - abs(dq)/acc;
    if disc < 0, ok = false; end
    tb(:) = td/2 - sqrt(max(disc, 0));
    a(1) = sign(dq)*acc;  a(2) = -a(1);
    if tb(1) > 0, v(1) = dq/(td - tb(1)); end
    tl = td - tb(1) - tb(2);               % time on the line
else
    % First segment: start at rest, hit qv(1) exactly
    dq = qv(2) - qv(1);
    a(1) = sign(dq)*acc;
    if a(1) ~= 0
        disc = td(1)^2 - 2*dq/a(1);
        if disc < 0, ok = false; end
        tb(1) = td(1) - sqrt(max(disc, 0));
        v(1)  = dq/(td(1) - tb(1)/2);
    end
    % Last segment: stop at rest, hit qv(n) exactly
    dq = qv(n) - qv(n-1);
    a(n) = sign(-dq)*acc;
    if a(n) ~= 0
        disc = td(end)^2 + 2*dq/a(n);
        if disc < 0, ok = false; end
        tb(n)  = td(end) - sqrt(max(disc, 0));
        v(n-1) = dq/(td(end) - tb(n)/2);
    end
    % Inner lines
    for j = 2:n-2
        v(j) = (qv(j+1) - qv(j))/td(j);
    end
    % Inner parabolas: change from one line velocity to the next
    for k = 2:n-1
        dv = v(k) - v(k-1);
        a(k)  = sign(dv)*acc;
        tb(k) = abs(dv)/acc;
    end
    % Time left on each line; the first and last parabola sit fully inside their segment
    tl = td - tb(1:n-1)/2 - tb(2:n)/2;
    tl(1)   = tl(1)   - tb(1)/2;
    tl(end) = tl(end) - tb(n)/2;
end
if any(tl < -1e-9), ok = false; end
tl = max(tl, 0);

% Build the pieces in time order: parabola 1, line 1, parabola 2, ..., parabola n
dur = zeros(1, 2*n-1);  acc_p = zeros(1, 2*n-1);
dur(1:2:end) = tb;  acc_p(1:2:end) = a;
dur(2:2:end) = tl;
t0 = [0, cumsum(dur)];                     % start time of each piece

% Integrate position and velocity from piece to piece
q0 = zeros(1, numel(dur));  v0 = zeros(1, numel(dur));
qc = qv(1);  vc = 0;
for p = 1:numel(dur)
    q0(p) = qc;  v0(p) = vc;
    if mod(p, 2) == 0, vc = v(p/2); v0(p) = vc; end   % a line moves at its own velocity
    qc = qc + vc*dur(p) + acc_p(p)*dur(p)^2/2;
    vc = vc + acc_p(p)*dur(p);
end

% Evaluate
q = qv(end)*ones(size(tt));  qd = zeros(size(tt));  qdd = zeros(size(tt));
for p = 1:numel(dur)
    in = tt >= t0(p) & tt < t0(p+1);
    tau = tt(in) - t0(p);
    q(in)   = q0(p) + v0(p)*tau + acc_p(p)*tau.^2/2;
    qd(in)  = v0(p) + acc_p(p)*tau;
    qdd(in) = acc_p(p);
end
end
