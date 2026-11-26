%% Step 7 — the same task in Simscape Multibody, compared with the MATLAB model
% Project requirements 6 (Simscape model), 7 (carry out the task with both
% models) and 8 (compare the motor torques).
%
% The model is built by build_rprr_simscape.m with the method of the 10/19
% exercise: the joint motions of step 4 go in, Simscape computes the joint
% torques, and they are compared with the ones of step 5 (RPRRdyn).
%
% This first version uses the same point masses as the MATLAB model. The two
% results must then agree almost exactly: a difference would mean an error in
% our equations, in the frames or in the units. Solid bodies and the CAD
% parts come later; then differences are expected and must be explained.
% after CAD edition 1 (2026-11-24): the point masses are now those of CAD version 1
% (robot_masses.m); point 6 is the slider, on the rail behind the q3 joint.

clear; close all; clc;

%% Settings
mdl      = 'robot_rprr';
payloads = [1.0 0.2];            % [kg] payloads run in Simscape (the two ends of the range)

%% Inputs
here = fileparts(mfilename('fullpath'));
S4 = load(fullfile(here, 'step4_trajectory.mat'));
S5 = load(fullfile(here, 'step5_dynamics.mat'));
L  = S4.L;
t  = S4.traj.t(:);   ns = numel(t);
to_si = [1; 1e-3; 1; 1];
Q = S4.traj.Q.*to_si;   Qd = S4.traj.Qd.*to_si;   Qdd = S4.traj.Qdd.*to_si;   % SI: rad, m

%% Build the model (always from scratch, so it cannot drift from the scripts)
build_rprr_simscape(mdl, here);
load_system(fullfile(here, [mdl '.slx']));

%% Run it for each payload
Fsim = zeros(4, ns, numel(payloads));  Fmat = Fsim;  tcp = zeros(3, ns);
for a = 1:numel(payloads)
    D  = robot_masses(payloads(a));
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('h_m', L.h/1000).setVariable('alpha_deg', rad2deg(L.alpha));
    in = in.setVariable('L3_m', L.L3/1000).setVariable('L4_m', L.L4/1000);
    in = in.setVariable('J1', D.J1).setVariable('m_pts', D.m).setVariable('g', D.g).setVariable('t_out', t);
    in = in.setVariable('s_m6', D.pts(D.i_slider,3));           % slider CoM along the rail (after CAD edition 1, 2026-11-24)
    for j = 1:4
        in = in.setVariable(sprintf('q_%d', j), [t, Q(j,:).']);       % [time, value], as on Ex4.2 p.16
        in = in.setVariable(sprintf('w_%d', j), [t, Qd(j,:).']);
        in = in.setVariable(sprintf('b_%d', j), [t, Qdd(j,:).']);
    end
    out = sim(in);
    for j = 1:4
        ts = out.(sprintf('F_q%d', j));
        Fsim(j,:,a) = interp1(ts.Time, squeeze(ts.Data), t).';
    end
    if a == 1
        tcp = [interp1(out.tcp_x.Time, squeeze(out.tcp_x.Data), t), interp1(out.tcp_y.Time, squeeze(out.tcp_y.Data), t), ...
               interp1(out.tcp_z.Time, squeeze(out.tcp_z.Data), t)].';
    end
    % MATLAB model for the same payload (pieces saved by step 5)
    Fmat(:,:,a) = S5.dyn.Fin0 + payloads(a)*S5.dyn.Finu + S5.dyn.Fgr0 + payloads(a)*S5.dyn.Fgru;
end
close_system(mdl, 0);

%% Checks
% 1. the pen tip of the Simscape model against direct kinematics (and so against the letter)
Tip = zeros(3, ns);
for i = 1:ns, Tip(:,i) = RPRRdir(S4.traj.Q(:,i), L)/1000; end
err_tip = max(vecnorm(tcp - Tip))*1000;                       % [mm]
% 2. joint torques: Simscape against MATLAB
dF   = Fsim - Fmat;
errF = squeeze(max(abs(dF), [], 2));                           % [4 x payloads]
peak = squeeze(max(abs(Fmat), [], 2));
unit_F = ["N m", "N", "N m", "N m"];
fprintf('Simscape model %s.slx: 4 joints, J1 and %d point masses; trajectory of step 4 (%.2f s, %d samples)\n', mdl, numel(robot_masses().m), t(end), ns);
fprintf('\nPen tip, Simscape against RPRRdir: largest distance %.2e mm\n', err_tip);
fprintf('\nJoint loads, Simscape against the MATLAB model (RPRRdyn):\n');
fprintf('  %-8s %-10s %14s %18s %14s\n', 'payload', 'joint', 'peak (MATLAB)', 'largest difference', 'relative');
for a = 1:numel(payloads)
    for j = 1:4
        fprintf('  %4.1f kg  q%d %19.3f %-3s %12.2e %-3s %12.1e\n', payloads(a), j, peak(j,a), unit_F(j), errF(j,a), unit_F(j), errF(j,a)/peak(j,a));
    end
end
assert(err_tip < 1e-3, 'The Simscape pen tip does not follow the MATLAB one: frames, zero positions or units differ');
assert(all(errF./peak < 1e-3, 'all'), 'Simscape and MATLAB joint loads differ by more than 0.1%%');

%% Figure 1 — joint loads from both models
fig1 = figure('Name', 'Step 7 — Simscape vs MATLAB', 'Color', 'w', 'Position', [100 100 1200 760]);
names = ["q1 (base turn)", "q2 (slider)", "q3", "q4 (pen angle)"];
for j = 1:4
    subplot(4, 2, 2*j - 1); hold on; grid on;
    plot(t, Fmat(j,:,1), '-', 'Color', [0.2 0.45 0.8], 'LineWidth', 2.4);
    plot(t, Fsim(j,:,1), '-', 'Color', [0.9 0.55 0.1], 'LineWidth', 0.9);
    ylabel(sprintf('%s [%s]', names(j), unit_F(j))); xlim([0 t(end)]);
    if j == 1, title(sprintf('Joint loads, payload %.1f kg', payloads(1))); legend({'MATLAB (RPRRdyn)', 'Simscape'}, 'Location', 'best'); end
    if j == 4, xlabel('t [s]'); end
    subplot(4, 2, 2*j); hold on; grid on;
    plot(t, dF(j,:,1), 'k-');
    ylabel(sprintf('[%s]', unit_F(j))); xlim([0 t(end)]);
    if j == 1, title('Simscape minus MATLAB'); end
    if j == 4, xlabel('t [s]'); end
end
exportgraphics(fig1, fullfile(here, 'figures', 'step7_torques.png'), 'Resolution', 150);

%% Save
result = struct('t', t.', 'payloads', payloads, 'Fsim', Fsim, 'Fmat', Fmat, 'tcp', tcp, 'err_tip_mm', err_tip, 'errF', errF);
save(fullfile(here, 'step7_simscape.mat'), 'result');
fprintf('\nSaved %s.slx, step7_simscape.mat and figures/step7_torques.png\n', mdl);
