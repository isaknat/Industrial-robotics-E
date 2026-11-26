%% Step 5 — dynamic model, motor torques, choice of motors and reductions
% Project requirements 1 (dynamic model), 9 (motor torques against the motor
% performance curve) and 10 (payload 0.2-1 kg and cycle time).
%
% 1. Inverse dynamics along the step 4 trajectory: what each joint must deliver
%    (Lez_03 p.4-7, method of the handout "SCARA Dynamics", links as point masses).
% 2. For each joint, choose a motor and a reduction ratio with the method of
%    10/22 (Lez_08 MotorSizingNotes p.21-28): load factor beta, accelerating
%    factor alpha, usable range of the transmission ratio tau.
% 3. Check the chosen motors: RMS torque <= rated, peak torque <= maximum,
%    speed <= maximum (Lez_08 p.10), and draw the check of Lez_08 p.29.
% 4. Repeat the check for payloads from 0.2 to 1 kg and for shorter and longer
%    cycle times.
%
% Units: this script and the RPRR*din / RPRRdyn functions work in SI (m, rad,
% kg, N, N m). The trajectory of step 4 is in mm and is converted here.
% tau = joint speed / motor speed, as in the lectures: no unit for q1, q3, q4;
% metres per radian for the slider q2 (Lez_08 p.32: "M instead of J").
%
% after CAD edition 1 (2026-11-24): the masses now come from CAD version 1 (robot_masses.m,
% step6_cad_masses.json). The slider is a point of its own behind the q3 joint, so the
% "at rest" test below counts only the points that turn with q3. Results with the
% estimated masses: the archive of 2026-11-24 (not part of this package).

clear; close all; clc;

%% Settings
payload_design = 1.0;                 % [kg] payload used to choose the motors: the heaviest of the range
payloads   = 0.2:0.2:1.0;             % [kg] payloads compared (requirement 10)
scales     = logspace(log10(0.2), log10(2), 80);   % cycle time / cycle time of step 4 (requirement 10)
use_max    = 0.8;                     % use at most this share of a motor's rated and maximum torque:
                                      % the masses are still guesses                              PLACEHOLDER
i_list     = [1 2 3 4 5 6 8 10 12 15 16 20 25 30 36 40 50];   % reductions 1:i we expect to be able to build
                                      % ourselves (belts, gears, printed cycloidal); 1:50 at most  PLACEHOLDER
lead_list  = [2 4 5 8 10 16 20]*1e-3; % [m per motor turn] for the slider (screw lead, or belt pulley
                                      % circumference)                                            PLACEHOLDER
motor_order = ["HG-KR053" "HG-MR053" "HG-KR13" "HG-MR13" "HG-KR23" "HG-MR23"];   % smallest first
% Choices already made. "" = let the script choose. Ratio: 1:i for q1, q3, q4; mm per motor turn for q2.
% DECIDED 2026-11-21: q3 = 100 W motor with 1:25 (option B of step5b_q3_options).
% DECIDED 2026-11-22: q2 = screw with 10 mm lead; q4 = 1:6 with its motor moved to the root of
% the q3 link (see robot_masses); q1 stays as the script first chose it (50 W, 1:50).
% All four are now fixed, so a change of masses cannot silently change a motor.
% after CAD edition 3, DECIDED 2026-11-26: q3 = 50 W motor (HG-KR053) with 1:42.19 = 90/16 * 120/16, stock
% pulleys on the unchanged shafts (before: 100 W HG-KR13 with 1:25). Possible because the third
% edition took the q4 motor off the arm. The q4 motor is on the slider since the third edition.
fix_motor  = ["HG-KR053" "HG-KR053" "HG-KR053" "HG-KR053"];
fix_ratio  = [50 10 90/16*120/16 6];

%% Inputs
here = fileparts(mfilename('fullpath'));
S4 = load(fullfile(here, 'step4_trajectory.mat'));
L  = S4.L;
Lm = L;  Lm.h = L.h/1000;  Lm.L3 = L.L3/1000;  Lm.L4 = L.L4/1000;     % dimensions in metres
t  = S4.traj.t;   ns = numel(t);   Tc = t(end);
to_si = [1; 1e-3; 1; 1];
Q = S4.traj.Q.*to_si;   Qd = S4.traj.Qd.*to_si;   Qdd = S4.traj.Qdd.*to_si;
D  = robot_masses(payload_design);
D0 = robot_masses(0);                                  % robot without payload
Du = D0;  Du.J1 = 0;  Du.m = zeros(size(D0.m));  Du.m(D0.i_tcp) = 1;   % 1 kg at the pen tip and nothing else
motors = motor_catalog();
[~, ord] = ismember(motor_order, [motors.name]);
motors = motors(ord);
joint_names = ["q1 (base turn)", "q2 (slider)", "q3", "q4 (pen angle)"];
unit_F = ["N m", "N", "N m", "N m"];

%% 1. Tests of the dynamic functions
rng(1);
errJ = 0;  errJp = 0;
for k = 1:50
    q  = [pi*(2*rand - 1); 0.03 + 0.15*rand; 1.5*(2*rand - 1); 1.5*(2*rand - 1)];
    qp = 2*rand(4, 1) - 1;
    % (a) extended Jacobian against differences of the positions (the method of Lez_10 p.35)
    Je = RPRRjacdin(q, Lm, D);  Jn = zeros(size(Je));  d = 1e-6;
    for j = 1:4
        e = zeros(4, 1);  e(j) = d;
        Jn(:,j) = (RPRRdirdin(q + e, Lm, D) - RPRRdirdin(q - e, Lm, D))/(2*d);
    end
    errJ = max(errJ, max(abs(Je - Jn), [], 'all'));
    % (b) its time derivative against a difference of the Jacobian along the motion
    Jpn = (RPRRjacdin(q + d*qp, Lm, D) - RPRRjacdin(q - d*qp, Lm, D))/(2*d);
    errJp = max(errJp, max(abs(RPRRjacPdin(q, qp, Lm, D) - Jpn), [], 'all'));
end
% (c) at rest the joints only hold the weight: compare with weight x lever arm, worked out directly
q = Q(:,1);  Se = RPRRdirdin(q, Lm, D);
P = reshape(Se(2:end), 3, []);  rho = vecnorm(P(1:2,:));           % point masses: distance from the axis
F_rest = RPRRdyn(q, zeros(4,1), zeros(4,1), Lm, D);
on_tool = D.pts(:,2) > 0;                                           % points beyond the q4 joint
on_q3 = true(size(D.m(:)));  on_q3(D.i_slider) = false;             % points that turn with q3 (not the slider)
hand = [0;
        D.g*sum(D.m)*sin(Lm.alpha);                                 % weight component along the rail
        D.g*sum(D.m(on_q3).'.*(rho(on_q3) - rho(1)));               % moment of the weights on the q3 link about the q3 joint
        D.g*sum(D.m(on_tool).'.*(rho(on_tool) - rho(3)))];          % moment of the tool and payload about the q4 joint
err_static = max(abs(F_rest - hand));

%% 2. Inverse dynamics along the trajectory
% The joint loads are linear in the masses, and the "moving" part scales with
% 1/time^2. So four runs are enough for every payload and every cycle time:
% robot without payload and 1 kg at the tip, each split into moving and weight parts.
Fin0 = zeros(4, ns);  Fgr0 = Fin0;  Finu = Fin0;  Fgru = Fin0;
Mdiag = zeros(4, ns);  Energy = zeros(1, ns);  Fq_direct = Fin0;
C = eye(4); C(4,3) = 0.5;
MdriveDiag = zeros(4,ns);
Mv = [D.J1; kron(D.m(:), [1; 1; 1])];
for i = 1:ns
    [~, Fin0(:,i), Fgr0(:,i)] = RPRRdyn(Q(:,i), Qd(:,i), Qdd(:,i), Lm, D0);
    [~, Finu(:,i), Fgru(:,i)] = RPRRdyn(Q(:,i), Qd(:,i), Qdd(:,i), Lm, Du);
    [Fq_direct(:,i), ~, ~, Mbar] = RPRRdyn(Q(:,i), Qd(:,i), Qdd(:,i), Lm, D);
    Mdiag(:,i) = diag(Mbar);
    MdriveDiag(:,i) = diag(C.'\Mbar/C);
    Se  = RPRRdirdin(Q(:,i), Lm, D);
    Sed = RPRRjacdin(Q(:,i), Lm, D)*Qd(:,i);
    Energy(i) = 0.5*sum(Mv.*Sed.^2) + D.g*sum(D.m.*Se(4:3:end));    % kinetic + potential
end
load_of = @(mp, s) (Fin0 + mp*Finu)/s^2 + Fgr0 + mp*Fgru;           % joint loads for payload mp, time scale s
Fq  = load_of(payload_design, 1);
Fin = Fin0 + payload_design*Finu;   Fgr = Fgr0 + payload_design*Fgru;
err_split = max(abs(Fq - Fq_direct), [], 'all');
% (d) energy: the work done by the joints must equal the change of kinetic + potential energy
Work = cumtrapz(t, sum(Fq.*Qd, 1));
err_energy = max(abs(Work - (Energy - Energy(1))));
energy_span = max(Energy) - min(Energy);

%% 3. Motor and reduction for each joint (10/22, Lez_08 p.21-28)
% Virtual work gives Fdrive = C'\Fjoint for vdrive = C*vjoint.
Vdrive = C*Qd; Adrive = C*Qdd;
Fdrive = C.'\Fq; Gdrive = C.'\Fgr;
rmsf = @(x) sqrt(trapz(t, x.^2, 2)/Tc);
Crq  = rmsf(Fdrive);                       % RMS of the load C_r*
wdq  = rmsf(Adrive);                      % RMS of the joint acceleration
beta = 2*(wdq.*Crq + trapz(t, Adrive.*Fdrive, 2)/Tc);      % load factor [W/s]
w_max = max(abs(Vdrive), [], 2);
nm = numel(motors);
alpha = [motors.Cn].^2./[motors.Jm];   % accelerating factor of each motor [W/s]
tau_opt = zeros(4, nm);  tau_min = nan(4, nm);  tau_max = nan(4, nm);  tau_p = zeros(4, nm);
for j = 1:4
    for k = 1:nm
        Jm = motors(k).Jm;
        tau_opt(j,k) = sqrt(Jm*wdq(j)/Crq(j));
        tau_p(j,k)   = w_max(j)/motors(k).wmax;
        if alpha(k) > beta(j)
            r1 = sqrt(alpha(k) - beta(j) + 4*wdq(j)*Crq(j));  r2 = sqrt(alpha(k) - beta(j));
            tau_min(j,k) = sqrt(Jm)*(r1 - r2)/(2*Crq(j));
            tau_max(j,k) = sqrt(Jm)*(r1 + r2)/(2*Crq(j));
        end
    end
end

% Ratios we can build, and the choice: the smallest motor for which one of
% them passes the three checks with the margin use_max; among those ratios the
% one closest to tau_opt.
cand = {1./i_list, lead_list/(2*pi), 1./i_list, 1./i_list};
sel = struct('motor', cell(1, 4), 'tau', [], 'k', []);
for j = 1:4
    if fix_motor(j) ~= ""                                   % a choice already made: use it, but still check it
        k = find([motors.name] == fix_motor(j));
        if j == 2, tau = fix_ratio(j)*1e-3/(2*pi); else, tau = 1/fix_ratio(j); end
        sel(j).motor = motors(k);  sel(j).tau = tau;  sel(j).k = k;
        sel(j).use = motor_use(t, Fdrive(j,:), Vdrive(j,:), Adrive(j,:), tau, motors(k));
        assert(sel(j).use.rms <= use_max && sel(j).use.peak <= use_max && sel(j).use.speed <= 1, ...
               'The motor and ratio fixed for joint %d do not pass the checks with the margin', j);
        continue
    end
    for k = 1:nm
        okc = false(size(cand{j}));
        for c = 1:numel(cand{j})
            r = motor_use(t, Fdrive(j,:), Vdrive(j,:), Adrive(j,:), cand{j}(c), motors(k));
            okc(c) = r.rms <= use_max && r.peak <= use_max && r.speed <= 1;
        end
        if any(okc)
            tc = cand{j}(okc);
            [~, b] = min(abs(log(tc/tau_opt(j,k))));
            sel(j).motor = motors(k);  sel(j).tau = tc(b);  sel(j).k = k;
            break
        end
    end
    assert(~isempty(sel(j).motor), 'No motor of the list can drive joint %d with a ratio we can build', j);
    sel(j).use = motor_use(t, Fdrive(j,:), Vdrive(j,:), Adrive(j,:), sel(j).tau, sel(j).motor);
end
% The RMS check and the tau range of the slides must say the same thing
agree = true;
for j = 1:4
    for k = 1:nm
        for c = 1:numel(cand{j})
            r = motor_use(t, Fdrive(j,:), Vdrive(j,:), Adrive(j,:), cand{j}(c), motors(k));
            in_range = cand{j}(c) >= tau_min(j,k) && cand{j}(c) <= tau_max(j,k);    % false if no range (NaN)
            agree = agree && ((r.rms <= 1) == in_range || abs(r.rms - 1) < 1e-6);
        end
    end
end

% What the chosen motors could give at the joints (for the limits of step 4)
v_lim = zeros(4, 1);  a_lim = zeros(4, 1);
for j = 1:4
    m = sel(j).motor;  tau = sel(j).tau;
    v_lim(j) = m.wn*tau;                                                        % at rated speed
    a_lim(j) = (use_max*m.Cmax/tau - max(abs(Gdrive(j,:))))/(max(MdriveDiag(j,:)) + m.Jm/tau^2);
end

%% 4. Payload and cycle time (requirement 10), with the chosen motors
worst = zeros(numel(payloads), numel(scales));       % largest of the three ratios over the four joints
which = strings(numel(payloads), numel(scales));
for a = 1:numel(payloads)
    for b = 1:numel(scales)
        s = scales(b);  Fab = C.'\load_of(payloads(a), s);
        for j = 1:4
            r = motor_use(t, Fab(j,:), Vdrive(j,:)/s, Adrive(j,:)/s^2, sel(j).tau, sel(j).motor);
            [v, kind] = max([r.rms, r.peak, r.speed]);
            if v > worst(a,b)
                worst(a,b) = v;  kinds = ["RMS torque" "peak torque" "speed"];
                which(a,b) = sprintf('q%d %s', j, kinds(kind));
            end
        end
    end
end
T_min = nan(numel(payloads), 1);  lim_by = strings(numel(payloads), 1);
for a = 1:numel(payloads)
    b = find(worst(a,:) > 1, 1, 'last');             % shortest cycle from which every check holds
    if isempty(b), b = 0; end
    if b < numel(scales), T_min(a) = scales(b+1)*Tc; end
    if b >= 1, lim_by(a) = which(a,b); end
end

%% Summary
fprintf('Dynamic model: %d point masses + J1; payload %.1f kg; trajectory of step 4 (%.2f s, %d samples)\n', numel(D.m), payload_design, Tc, ns);
fprintf('Masses [kg]: slider %.3f, q3 body (link, long belt, q3 pulley) %.3f, pen head %.3f, payload %.2f; J1 = %.4f kg m^2\n   source: %s\n', ...
        D.m_slider, D.m_q3body, D.m_tool, D.m_payload, D.J1, D.source);
fprintf('\nTests (largest error):\n');
fprintf('  extended Jacobian vs differences of the positions   %.1e\n', errJ);
fprintf('  its time derivative vs differences of the Jacobian  %.1e\n', errJp);
fprintf('  at rest at Home: joints vs weight x lever arm       %.1e   (q3 holds %.2f N m, q4 %.3f N m)\n', err_static, F_rest(3), F_rest(4));
fprintf('  payload and time-scale shortcut vs direct run       %.1e\n', err_split);
fprintf('  work of the joints vs change of energy              %.1e J  (energy varies by %.2f J over the cycle)\n', err_energy, energy_span);
fprintf('  RMS check and tau range of the slides agree:        %s\n', tern(agree, 'yes', 'NO'));
fprintf('\nWhat the joints must deliver (payload %.1f kg):\n', payload_design);
fprintf('  %-16s %10s %10s %12s %14s\n', 'joint', 'peak', 'RMS', 'weight part', 'moving part');
for j = 1:4
    fprintf('  %-16s %7.3f %-3s %6.3f %-3s %8.3f %-3s %10.3f %-3s\n', joint_names(j), max(abs(Fq(j,:))), unit_F(j), Crq(j), unit_F(j), ...
            max(abs(Fgr(j,:))), unit_F(j), max(abs(Fin(j,:))), unit_F(j));
end
fprintf('\nLoad factor beta of each joint and accelerating factor alpha of each motor [W/s]:\n');
fprintf('  beta:  q1 %.3g, q2 %.3g, q3 %.3g, q4 %.3g\n', beta);
fprintf('  alpha: '); fprintf('%s %.0f   ', [[motors.name]; string(round(alpha))]); fprintf('\n');
fprintf('\nChoice (smallest motor with a ratio we can build, using at most %.0f%% of its torque; fixed by us: %s):\n', 100*use_max, ...
        strjoin(compose('q%d', find(fix_motor ~= "")), ', '));
fprintf('  %-16s %-9s %-22s %9s %9s %9s %12s\n', 'joint', 'motor', 'ratio', 'RMS/rated', 'peak/max', 'speed/max', 'motor speed');
for j = 1:4
    if j == 2, rs = sprintf('%.0f mm per motor turn', sel(j).tau*2*pi*1000); else, rs = sprintf('1:%.0f', 1/sel(j).tau); end
    u = sel(j).use;
    fprintf('  %-16s %-9s %-22s %9.2f %9.2f %9.2f %8.0f r/min\n', joint_names(j), sel(j).motor.name, rs, u.rms, u.peak, u.speed, u.speed*sel(j).motor.wmax*60/(2*pi));
end
fprintf('  usable range of tau from the slides for the chosen motor, and tau_opt:\n');
for j = 1:4
    k = sel(j).k;
    if j == 2
        fprintf('    q2: %.2f to %.0f mm per turn (best %.2f; speed needs at least %.2f)\n', [tau_min(j,k), tau_max(j,k), tau_opt(j,k), tau_p(j,k)]*2*pi*1000);
    else
        fprintf('    q%d: 1:%.0f to 1:%.1f (best 1:%.0f; speed allows up to 1:%.0f)\n', j, 1/tau_min(j,k), 1/tau_max(j,k), 1/tau_opt(j,k), 1/tau_p(j,k));
    end
end
fprintf('  least reduction each motor needs to pass the RMS check at 100%% of its rated torque (- = cannot):\n');
fprintf('    %-4s', ''); fprintf('%10s', erase([motors.name], "HG-")); fprintf('\n');
for j = 1:4
    fprintf('    q%d  ', j);
    for k = 1:nm
        if isnan(tau_max(j,k)), fprintf('%10s', '-');
        elseif j == 2, fprintf('%7.0f mm', tau_max(j,k)*2*pi*1000);
        else, fprintf('%10s', sprintf('1:%.1f', 1/tau_max(j,k))); end
    end
    fprintf('\n');
end
fprintf('\nWhat these motors could give at the joints (rated speed; %.0f%% of maximum torque minus the weight, largest inertia):\n', 100*use_max);
fprintf('  speed:        q1 %.1f rad/s, q2 %.0f mm/s, q3 %.1f rad/s, q4 %.1f rad/s      (step 4 uses %g, %g, %g, %g)\n', v_lim.*[1; 1000; 1; 1], L.vmax);
fprintf('  acceleration: q1 %.0f rad/s^2, q2 %.0f mm/s^2, q3 %.0f rad/s^2, q4 %.0f rad/s^2   (step 4 uses %g, %g, %g, %g)\n', a_lim.*[1; 1000; 1; 1], L.amax);
fprintf('\nPayload and cycle time with these motors (the same path, played faster or slower):\n');
fprintf('  %-10s %-34s %s\n', 'payload', 'at the cycle time of step 4', 'shortest cycle that passes every check');
i1 = find(scales >= 1, 1);
for a = 1:numel(payloads)
    Fa = C.'\load_of(payloads(a), 1);  rr = zeros(1, 4);
    for j = 1:4, r = motor_use(t, Fa(j,:), Vdrive(j,:), Adrive(j,:), sel(j).tau, sel(j).motor); rr(j) = max([r.rms, r.peak, r.speed]); end
    fprintf('  %4.1f kg    largest use %.2f (q%d)%16s %5.1f s   limited by %s\n', payloads(a), max(rr), find(rr == max(rr), 1), '', T_min(a), lim_by(a));
end
assert(max([errJ, errJp]) < 1e-6 && err_static < 1e-9 && err_split < 1e-9, 'A test of the dynamic functions failed');
assert(err_energy < 0.01*energy_span, 'Energy balance is off by more than 1%%');
assert(agree, 'The RMS check and the tau range disagree');

%% Figure 1 — joint loads over the cycle
fig1 = figure('Name', 'Step 5 — joint loads', 'Color', 'w', 'Position', [100 100 1100 700]);
tb = cumsum([S4.traj.T_arc, S4.traj.T_letter]);
for j = 1:4
    subplot(2, 2, j); hold on; grid on;
    plot(t, Fgr(j,:), '-', 'Color', [0.2 0.6 0.3], 'LineWidth', 1);
    plot(t, Fin(j,:), '-', 'Color', [0.9 0.55 0.1], 'LineWidth', 1);
    plot(t, Fq(j,:), 'k-', 'LineWidth', 1.3);
    xline(tb(1), 'k:'); xline(tb(2), 'k:');
    xlabel('t [s]'); ylabel(sprintf('[%s]', unit_F(j))); title(joint_names(j)); xlim([0 Tc]);
    if j == 1, legend({'holding the weight', 'moving the masses', 'total'}, 'Location', 'best'); end
end
sgtitle(sprintf('What each joint must deliver, payload %.1f kg (dotted: arc | letter | line)', payload_design));

%% Figure 2 — alpha against beta, and the usable ratios
fig2 = figure('Name', 'Step 5 — motor and ratio choice', 'Color', 'w', 'Position', [100 100 1200 760]);
for j = 1:4
    subplot(2, 4, j); hold on; grid on;
    bar(1:nm, alpha, 'FaceColor', [0.6 0.75 0.95]);
    if beta(j) > 0, yline(beta(j), 'r-', 'LineWidth', 1.5); end
    set(gca, 'YScale', 'log', 'XTick', 1:nm, 'XTickLabel', erase([motors.name], "HG-"), 'XTickLabelRotation', 60);
    ylim([min([alpha, max(beta(j), 1e-3)])/3, max(alpha)*3]);
    title(sprintf('%s', joint_names(j))); if j == 1, ylabel('\alpha (bars), \beta (red) [W/s]'); end
    subplot(2, 4, 4 + j); hold on; grid on;
    sc = 1;  if j == 2, sc = 2*pi*1000; end                % slider: show mm per motor turn
    for k = 1:nm
        if ~isnan(tau_min(j,k))
            plot([k k], [tau_min(j,k) tau_max(j,k)]*sc, '-', 'Color', [0.3 0.5 0.85], 'LineWidth', 5);
        end
        plot(k, tau_opt(j,k)*sc, 'ko', 'MarkerFaceColor', 'w');
        plot(k, tau_p(j,k)*sc, 'r_', 'MarkerSize', 12, 'LineWidth', 1.5);
    end
    yline(max(cand{j})*sc, ':');  yline(min(cand{j})*sc, ':');
    plot(sel(j).k, sel(j).tau*sc, 'p', 'MarkerSize', 13, 'MarkerFaceColor', [0.95 0.75 0.1], 'MarkerEdgeColor', 'k');
    set(gca, 'YScale', 'log', 'XTick', 1:nm, 'XTickLabel', erase([motors.name], "HG-"), 'XTickLabelRotation', 60);
    xlim([0.5 nm + 0.5]);
    if j == 2, ylabel('mm per motor turn'); elseif j == 1, ylabel('\tau = joint speed / motor speed'); end
end
sgtitle({'Top: a motor can do the job if its \alpha is above the joint''s \beta', ...
         'Bottom: usable \tau (blue), \tau_{opt} (circle), slowest allowed by motor speed (red), what we can build (dotted), choice (star)'});

%% Figure 3 — the chosen motors in the speed-torque plane (Lez_08 p.29)
fig3 = figure('Name', 'Step 5 — motor checks', 'Color', 'w', 'Position', [100 100 1100 760]);
for j = 1:4
    m = sel(j).motor;  tau = sel(j).tau;
    Cm = tau*Fdrive(j,:) + m.Jm*Adrive(j,:)/tau;   wm = Vdrive(j,:)/tau*60/(2*pi);
    subplot(2, 2, j); hold on; grid on;
    nmax = m.wmax*60/(2*pi);
    plot([-nmax nmax NaN -nmax nmax], [m.Cmax m.Cmax NaN -m.Cmax -m.Cmax], 'r-', 'LineWidth', 1.2);
    plot([-nmax nmax NaN -nmax nmax], [m.Cn m.Cn NaN -m.Cn -m.Cn], '-', 'Color', [0.2 0.6 0.3], 'LineWidth', 1.2);
    plot([-nmax nmax NaN -nmax nmax], sel(j).use.rms*m.Cn*[1 1 NaN -1 -1], 'b--');
    plot([nmax nmax NaN -nmax -nmax], [-m.Cmax m.Cmax NaN -m.Cmax m.Cmax], 'r--');
    plot(wm, Cm, 'k-', 'LineWidth', 1);
    xlim([-1.1 1.1]*nmax); ylim([-1.15 1.15]*m.Cmax);
    xlabel('motor speed [r/min]'); ylabel('motor torque [N m]');
    if j == 2, rs = sprintf('%.0f mm/turn', tau*2*pi*1000); else, rs = sprintf('1:%.0f', 1/tau); end
    title(sprintf('%s: %s, %s', joint_names(j), m.name, rs));
    if j == 1, legend({'maximum torque', 'rated torque', 'RMS of the cycle', 'maximum speed', 'the cycle'}, 'Location', 'northeast', 'FontSize', 7); end
end
sgtitle(sprintf('Chosen motors: the cycle in the speed-torque plane, payload %.1f kg', payload_design));

%% Figure 4 — payload and cycle time
fig4 = figure('Name', 'Step 5 — payload and cycle time', 'Color', 'w', 'Position', [100 100 800 500]);
hold on; grid on;
cols = parula(numel(payloads) + 1);
for a = 1:numel(payloads)
    plot(scales*Tc, worst(a,:), '-', 'Color', cols(a,:), 'LineWidth', 1.6);
end
yline(1, 'r-', 'LineWidth', 1.2);  yline(use_max, 'r--');  xline(Tc, 'k:');
set(gca, 'XScale', 'log', 'YScale', 'log'); xlim([scales(1) scales(end)]*Tc); ylim([0.1 10]);
xlabel('cycle time [s]'); ylabel('largest use of a motor (1 = at its limit)');
legend([compose('payload %.1f kg', payloads(:)); {'limit'; sprintf('%.0f%% margin', 100*use_max); 'cycle of step 4'}], 'Location', 'northeast');
title('Same path played faster or slower: worst of RMS/rated, peak/maximum, speed/maximum');

%% Save
dyn = struct('t', t, 'Fq', Fq, 'Fin', Fin, 'Fgr', Fgr, 'Fin0', Fin0, 'Fgr0', Fgr0, 'Finu', Finu, 'Fgru', Fgru, ...
             'payload', payload_design, 'beta', beta, 'Crq', Crq, 'wdq', wdq, 'Mdiag', Mdiag);
choice = struct('joint', num2cell(joint_names), 'motor', {sel.motor}, 'tau', {sel.tau}, 'use', {sel.use});
study  = struct('payloads', payloads, 'cycle', scales*Tc, 'worst', worst, 'T_min', T_min, 'limited_by', lim_by);
save(fullfile(here, 'step5_dynamics.mat'), 'dyn', 'choice', 'study', 'D', 'motors', 'alpha', 'tau_min', 'tau_max', 'tau_opt', 'tau_p', ...
     'v_lim', 'a_lim', 'use_max', 'i_list', 'lead_list');
figdir = fullfile(here, 'figures');
exportgraphics(fig1, fullfile(figdir, 'step5_joint_loads.png'), 'Resolution', 150);
exportgraphics(fig2, fullfile(figdir, 'step5_motor_choice.png'), 'Resolution', 150);
exportgraphics(fig3, fullfile(figdir, 'step5_motor_checks.png'), 'Resolution', 150);
exportgraphics(fig4, fullfile(figdir, 'step5_payload_cycle.png'), 'Resolution', 150);
fprintf('\nSaved step5_dynamics.mat and figures/step5_*.png\n');

%% Local functions
function s = tern(c, a, b)
if c, s = a; else, s = b; end
end
