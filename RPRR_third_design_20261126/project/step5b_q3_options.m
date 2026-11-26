%% Step 5b — three ways to drive q3, side by side
% q3 is the joint that carries the weight of everything beyond it, so it
% decides how much reduction we must build. Three options are compared on the
% same trajectory (step 4) and with the same method as step 5:
%   A  50 W motor (HG-KR053)                      -> needs the largest reduction
%   B  100 W motor (HG-KR13)                      -> smaller reduction, but 0.2 kg more on the slider
%   C  50 W motor + a spring that holds the weight (11/05, Lez_08_Components p.51)
% For each option: the smallest reduction that passes the three motor checks
% for every payload from 0.2 to 1 kg, how much of the motor is then used, what
% changes for the other joints, and the shortest cycle time that still passes.
%
% Nothing here changes the other steps: it only compares.
% DECIDED 2026-11-21: option B. It is now in robot_masses.m (slider mass with
% the 0.54 kg motor) and in step5_dynamics.m (fix_motor, fix_ratio). The slider
% mass of the other options is counted from that one: A and C are 0.2 kg lighter.
% after CAD edition 3, DECIDED 2026-11-26: changed to option A, the 50 W motor, built as 1:42.19 with stock
% pulleys (step5_dynamics.m). The slider mass in robot_masses is now with the 0.34 kg motor, and
% the other options are counted from it (base_motor below): B is 0.2 kg heavier.

clear; close all; clc;

%% Settings
payloads   = 0.2:0.2:1.0;        % [kg] every payload the motor must handle (requirement 10)
use_max    = 0.8;                % use at most this share of rated and maximum torque (as in step 5)
i_list     = [1 2 3 4 5 6 8 10 12 15 16 20 25 30 36 40 50];   % reductions 1:i we expect to be able to build
m_spring   = 0.6;                % [kg] payload for which the spring balances exactly: the middle of the range
spring_b   = 0.040;              % [m] example spring geometry (Lez_08_Components p.51): arm behind the joint
spring_c   = 0.060;              % [m] ... and fixed point below the joint. Only b*c*k matters.   PLACEHOLDER
scales     = logspace(log10(0.2), log10(2), 120);

opt = struct('name',  {"A: 50 W motor", "B: 100 W motor", "C: 50 W motor + spring"}, ...
             'motor', {"HG-KR053", "HG-KR13", "HG-KR053"}, ...
             'spring', {false, false, true});

%% Inputs
here = fileparts(mfilename('fullpath'));
S4 = load(fullfile(here, 'step4_trajectory.mat'));
S5 = load(fullfile(here, 'step5_dynamics.mat'));           % motors and ratios of the other joints
L  = S4.L;
Lm = L;  Lm.h = L.h/1000;  Lm.L3 = L.L3/1000;  Lm.L4 = L.L4/1000;
t  = S4.traj.t;  ns = numel(t);  Tc = t(end);
to_si = [1; 1e-3; 1; 1];
Q = S4.traj.Q.*to_si;  Qd = S4.traj.Qd.*to_si;  Qdd = S4.traj.Qdd.*to_si;
motors = motor_catalog();
base_motor = S5.choice(3).motor;                            % the motor whose mass is already in robot_masses
th3 = Lm.alpha + Q(3,:);                                    % angle of the q3 link above horizontal

% Spring (Lez_08_Components p.51): a spring from a point on the link, b behind
% the joint, to a fixed point c below the joint gives the torque k*b*c*cos(th3)
% on the link, the same shape as the weight's torque. With k*b*c = g*sum(m*a)
% (a = distance of each mass from the joint along the link) the link stays in
% balance at every angle. Both ends of the spring ride on the slider, so only
% q3 feels it. The tool is short, so its masses are counted at the q4 joint.
D6 = robot_masses(m_spring);
K_spring = D6.g*sum(D6.m(:).*D6.pts(:,1))*Lm.L3;            % [N m] = k*b*c
k_spring = K_spring/(spring_b*spring_c);                    % [N/m] for the example geometry

%% Joint loads for every option and payload
np = numel(payloads);
for v = 1:numel(opt)
    m3 = motors([motors.name] == opt(v).motor);
    opt(v).m3 = m3;
    opt(v).extra = m3.mass - base_motor.mass;               % heavier motor rides on the slider
    opt(v).Fin = zeros(4, ns, np);  opt(v).Fgr = zeros(4, ns, np);
    for a = 1:np
        D = robot_masses(payloads(a));
        D.m(D.i_slider) = D.m(D.i_slider) + opt(v).extra;   % after CAD edition 1 (2026-11-24): the slider has its own point
        for i = 1:ns
            [~, opt(v).Fin(:,i,a), opt(v).Fgr(:,i,a)] = RPRRdyn(Q(:,i), Qd(:,i), Qdd(:,i), Lm, D);
        end
        if opt(v).spring
            opt(v).Fgr(3,:,a) = opt(v).Fgr(3,:,a) - K_spring*cos(th3);
        end
    end
end

% Reduced-coordinate wrist drive and its work-conjugate loads.
C = eye(4); C(4,3) = 0.5;
Vdrive = C*Qd; Adrive = C*Qdd;
for v = 1:numel(opt)
    for a = 1:np
        opt(v).Fin(:,:,a) = C.'\opt(v).Fin(:,:,a);
        opt(v).Fgr(:,:,a) = C.'\opt(v).Fgr(:,:,a);
    end
end

%% For each option: smallest reduction of q3 that works for every payload
for v = 1:numel(opt)
    opt(v).i3 = NaN;
    for ii = i_list
        ok = true;
        for a = 1:np
            F3 = opt(v).Fin(3,:,a) + opt(v).Fgr(3,:,a);
            r = motor_use(t, F3, Qd(3,:), Qdd(3,:), 1/ii, opt(v).m3);
            ok = ok && r.rms <= use_max && r.peak <= use_max && r.speed <= 1;
        end
        if ok, opt(v).i3 = ii; break; end
    end
    assert(~isnan(opt(v).i3), 'Option %s: no reduction of the list works', opt(v).name);
    % use of every motor at every payload, and the shortest cycle that passes (limit 1.0)
    opt(v).use = zeros(4, np, 3);  opt(v).Tmin = zeros(1, np);  opt(v).hold3 = zeros(1, np);
    for a = 1:np
        worst = zeros(size(scales));
        for j = 1:4
            if j == 3, m = opt(v).m3; tau = 1/opt(v).i3; else, m = S5.choice(j).motor; tau = S5.choice(j).tau; end
            r = motor_use(t, opt(v).Fin(j,:,a) + opt(v).Fgr(j,:,a), Vdrive(j,:), Adrive(j,:), tau, m);
            opt(v).use(j,a,:) = [r.rms, r.peak, r.speed];
            for b = 1:numel(scales)
                s = scales(b);
                r = motor_use(t, opt(v).Fin(j,:,a)/s^2 + opt(v).Fgr(j,:,a), Vdrive(j,:)/s, Adrive(j,:)/s^2, tau, m);
                worst(b) = max([worst(b), r.rms, r.peak, r.speed]);
            end
        end
        b = find(worst > 1, 1, 'last');  if isempty(b), b = 0; end
        opt(v).Tmin(a) = scales(min(b + 1, numel(scales)))*Tc;
        opt(v).hold3(a) = max(abs(opt(v).Fgr(3,:,a)));       % largest torque q3 must hold (weight minus spring)
    end
end

%% Summary
fprintf('q3 compared for payloads %.1f to %.1f kg on the step 4 trajectory (%.2f s); margin: at most %.0f%% of rated and maximum torque\n\n', ...
        payloads(1), payloads(end), Tc, 100*use_max);
fprintf('%-26s %-10s %-9s %-22s %-22s %-20s %s\n', 'option', 'q3 motor', 'reduction', 'RMS/rated at 0.2-1 kg', 'peak/max at 0.2-1 kg', 'q3 holds up to', 'shortest cycle at 0.2 / 1 kg');
for v = 1:numel(opt)
    u = squeeze(opt(v).use(3,:,:));
    fprintf('%-26s %-10s 1:%-7d %.2f to %.2f %10s %.2f to %.2f %9s %5.2f N m %8s %.1f / %.1f s\n', opt(v).name, opt(v).motor, opt(v).i3, ...
            min(u(:,1)), max(u(:,1)), '', min(u(:,2)), max(u(:,2)), '', max(opt(v).hold3), '', opt(v).Tmin(1), opt(v).Tmin(end));
end
fprintf('\nOther joints (motors and ratios of step 5), largest use over all payloads [RMS/rated, peak/max]:\n');
for v = 1:numel(opt)
    u = opt(v).use;
    fprintf('  %-26s q1 %.2f, %.2f   q2 %.2f, %.2f   q4 %.2f, %.2f   (slider carries %+.2f kg)\n', opt(v).name, ...
            max(u(1,:,1)), max(u(1,:,2)), max(u(2,:,1)), max(u(2,:,2)), max(u(4,:,1)), max(u(4,:,2)), opt(v).extra);
end
fprintf('\nSpring of option C: k*b*c = %.2f N m, balanced exactly for a %.1f kg payload\n', K_spring, m_spring);
fprintf('  example: b = %.0f mm, c = %.0f mm -> k = %.2f N/mm; spring length %.0f to %.0f mm, force up to %.0f N\n', ...
        spring_b*1000, spring_c*1000, k_spring/1000, abs(spring_c - spring_b)*1000, (spring_c + spring_b)*1000, k_spring*(spring_c + spring_b));
fprintf('  the formula assumes a spring whose force is k times its full length (zero free length)\n');
fprintf('  torque q3 still has to hold with the spring: ');
fprintf('%.2f N m at %.1f kg   ', [opt(3).hold3; payloads]);  fprintf('\n');

%% Figure
fig = figure('Name', 'Step 5b — q3 options', 'Color', 'w', 'Position', [100 100 1300 430]);
cols = {[0.2 0.45 0.8], [0.9 0.55 0.1], [0.2 0.6 0.3]};
subplot(1, 3, 1); hold on; grid on;
for v = 1:numel(opt)
    plot(t, opt(v).Fin(3,:,end) + opt(v).Fgr(3,:,end), '-', 'Color', cols{v}, 'LineWidth', 3.4 - 1.1*v);   % A thick, so it shows under B
end
yline(0, 'k-'); xlabel('t [s]'); ylabel('q3 joint torque [N m]'); xlim([0 Tc]);
title(sprintf('What q3 must deliver, payload %.1f kg', payloads(end)));
legend([opt.name], 'Location', 'best');
subplot(1, 3, 2); hold on; grid on;
for v = 1:numel(opt)
    plot(payloads, opt(v).use(3,:,1), '-o', 'Color', cols{v}, 'LineWidth', max(1.4, 5.5 - 2*v), 'MarkerSize', 13 - 3*v, 'MarkerFaceColor', cols{v});
end
yline(use_max, 'r--'); yline(1, 'r-'); ylim([0 1.1]);
xlabel('payload [kg]'); ylabel('q3 motor: RMS torque / rated torque');
title({'Use of the q3 motor at the chosen reduction', '(A and B coincide)'});
subplot(1, 3, 3); hold on; grid on;
b = bar(1:numel(opt), [opt.i3], 'FaceColor', 'flat');
for v = 1:numel(opt), b.CData(v,:) = cols{v}; text(v, opt(v).i3 + 1.5, sprintf('1:%d', opt(v).i3), 'HorizontalAlignment', 'center'); end
set(gca, 'XTick', 1:numel(opt), 'XTickLabel', {'A', 'B', 'C'}); ylim([0 max([opt.i3]) + 8]);
ylabel('reduction 1:i'); title('Smallest reduction that passes for every payload');
sgtitle('Three ways to drive q3');

%% Save
results = rmfield(opt, {'Fin', 'Fgr'});
save(fullfile(here, 'step5b_q3_options.mat'), 'results', 'payloads', 'use_max', 'i_list', 'K_spring', 'k_spring', 'm_spring', 'spring_b', 'spring_c');
exportgraphics(fig, fullfile(here, 'figures', 'step5b_q3_options.png'), 'Resolution', 150);
fprintf('\nSaved step5b_q3_options.mat and figures/step5b_q3_options.png\n');
