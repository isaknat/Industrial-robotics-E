function build_rprr_simscape(mdl, folder)
% Builds the Simscape Multibody model of the R-P-R-R robot from scratch and
% saves it as <folder>/<mdl>.slx (Project requirement 6). Building it by script
% keeps it in step with the MATLAB model: same frames, zero positions and signs
% as RPRRdir, same point masses as robot_masses.
%
% Method of the 10/19 exercise (Ex4.2, SCARA): every joint has its motion
% "provided by input" and its torque "automatically computed"; the motion comes
% from three From Workspace blocks per joint (position, speed, acceleration)
% through a Simulink-PS Converter; the computed torque is sensed and saved.
%
% Numbers are not written into the blocks. They are read from variables that
% step7_simscape sets before each run (SI units):
%   h_m, alpha_deg, L3_m, L4_m     dimensions [m, deg]
%   J1, m_pts(1..6), g             masses as in robot_masses, gravity
%   s_m6                           CoM of the slider along the rail from the q3 joint [m]
%                                  (after CAD edition 1, 2026-11-24: point 6 was the q4 motor on the
%                                  q3 link; since CAD version 1 it is the slider's own point)
%   q_1..q_4, w_1..w_4, b_1..b_4   [t, value] position, speed, acceleration of each joint
%   t_out                          times at which results are saved
%
% Frames. Simscape joints always act along their own z axis, so frames are
% turned before each joint:
%   world: z up, robot axis = z, arm plane along +y when q1 = 0 (as in RPRRdir)
%   q1  revolute about world z
%   q2  prismatic along the rail: frame turned about x by (alpha - 90 deg), so its z lies along the rail
%   q3  revolute about the horizontal axis square to the arm plane: frame turned about y by 90 deg.
%       In that frame the q3 link points along -x when q3 = 0, and positive q3 tilts it up.
%   q4  revolute about the same horizontal axis, at the end of the q3 link

if bdIsLoaded(mdl), close_system(mdl, 0); end
old = fullfile(folder, [mdl '.slx']);
if exist(old, 'file'), delete(old); end          % rebuilt from scratch every time
load_system('sm_lib');  load_system('nesl_utility');
new_system(mdl);
S2P = ['nesl_utility/Simulink-PS' newline 'Converter'];
P2S = ['nesl_utility/PS-Simulink' newline 'Converter'];
RT  = 'sm_lib/Frames and Transforms/Rigid Transform';
INR = 'sm_lib/Body Elements/Inertia';
pos = @(col, row) [60 + 150*col, 60 + 95*row, 60 + 150*col + 70, 60 + 95*row + 50];
add = @(lib, name, col, row, varargin) add_block(lib, [mdl '/' name], 'Position', pos(col, row), varargin{:});

% --- world, gravity, solver
add('sm_lib/Frames and Transforms/World Frame', 'World', 0, 3);
add('sm_lib/Utilities/Mechanism Configuration', 'Gravity', 0, 4, 'GravityVector', '[0 0 -g]');
add(['nesl_utility/Solver' newline 'Configuration'], 'Solver', 0, 5);

% --- joints: motion provided by input, torque computed and sensed
jtype = {'Revolute', 'Prismatic', 'Revolute', 'Revolute'};  junit = {'rad', 'm', 'rad', 'rad'};
jcol  = [1 4 6 9];
for j = 1:4
    jn = sprintf('q%d', j);
    add(['sm_lib/Joints/' jtype{j} ' Joint'], jn, jcol(j), 3, ...
        'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque', 'SenseTorqueForce', 'on');
    names = {'q', 'w', 'b'};
    for k = 1:3
        add('simulink/Sources/From Workspace', sprintf('%s_%d', names{k}, j), jcol(j) - 1 + 0.45*(k - 1), 0 + 0.0*k, ...
            'VariableName', sprintf('%s_%d', names{k}, j), 'SampleTime', '0', 'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
        set_param([mdl '/' sprintf('%s_%d', names{k}, j)], 'Position', [60 + 150*(jcol(j) - 1) + 55*(k - 1), 30 + 45*(k - 1), 60 + 150*(jcol(j) - 1) + 55*(k - 1) + 45, 30 + 45*(k - 1) + 28]);
    end
    add(S2P, sprintf('in_q%d', j), jcol(j), 1.6, 'FilteringAndDerivatives', 'provide', 'UdotUserProvided', '2', 'Unit', junit{j});
    for k = 1:3
        line_sig(mdl, sprintf('%s_%d', names{k}, j), 1, sprintf('in_q%d', j), k);
    end
    con(mdl, sprintf('in_q%d', j), 'RConn', 1, jn, 'LConn', 2);
    add(P2S, sprintf('out_q%d', j), jcol(j) + 0.2, 5);
    add('simulink/Sinks/To Workspace', sprintf('F_q%d', j), jcol(j) + 0.2, 6, 'VariableName', sprintf('F_q%d', j), 'SaveFormat', 'Timeseries');
    con(mdl, jn, 'RConn', 2, sprintf('out_q%d', j), 'LConn', 1);
    line_sig(mdl, sprintf('out_q%d', j), 1, sprintf('F_q%d', j), 1);
end

% --- frames between the joints
add(RT, 'to shoulder', 2, 3, 'TranslationMethod', 'StandardAxis', 'TranslationStandardAxis', '+Z', 'TranslationStandardOffset', 'h_m');
add(RT, 'rail direction', 3, 3, 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+X', 'RotationAngle', 'alpha_deg - 90');
add(RT, 'q3 axis', 5, 3, 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+Y', 'RotationAngle', '90');
add(RT, 'q3 link', 8, 3, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[-L3_m 0 0]');
add(RT, 'half q3 link', 7, 2, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[-L3_m/2 0 0]');
add(RT, 'tool', 10.4, 3, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[-L4_m 0 0]');
add(RT, 'half tool', 10.4, 2, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[-L4_m/2 0 0]');

con(mdl, 'World', 'RConn', 1, 'q1', 'LConn', 1);
con(mdl, 'World', 'RConn', 1, 'Gravity', 'RConn', 1);
con(mdl, 'World', 'RConn', 1, 'Solver', 'RConn', 1);
con(mdl, 'q1', 'RConn', 1, 'to shoulder', 'LConn', 1);
con(mdl, 'to shoulder', 'RConn', 1, 'rail direction', 'LConn', 1);
con(mdl, 'rail direction', 'RConn', 1, 'q2', 'LConn', 1);
con(mdl, 'q2', 'RConn', 1, 'q3 axis', 'LConn', 1);
con(mdl, 'q3 axis', 'RConn', 1, 'q3', 'LConn', 1);
con(mdl, 'q3', 'RConn', 1, 'q3 link', 'LConn', 1);
con(mdl, 'q3', 'RConn', 1, 'half q3 link', 'LConn', 1);
con(mdl, 'q3 link', 'RConn', 1, 'q4', 'LConn', 1);
con(mdl, 'q4', 'RConn', 1, 'tool', 'LConn', 1);
con(mdl, 'q4', 'RConn', 1, 'half tool', 'LConn', 1);

% --- masses: J1 turning with q1, then the five point masses of robot_masses
add(INR, 'J1', 1.5, 4.4, 'InertiaType', 'Custom', 'Mass', '1e-9', 'MomentsOfInertia', '[J1/2 J1/2 J1]');
con(mdl, 'q1', 'RConn', 1, 'J1', 'RConn', 1);
add(RT, 'to slider CoM', 4.6, 5.4, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[0 0 s_m6]');
con(mdl, 'q2', 'RConn', 1, 'to slider CoM', 'LConn', 1);          % after q2 the frame's z runs along the rail
mass_at = {'q2', 'half q3 link', 'q3 link', 'half tool', 'tool', 'to slider CoM'};
mass_nm = {'m1 q3 joint', 'm2 mid q3 link', 'm3 q4 joint', 'm4 mid tool', 'm5 tip + payload', 'm6 slider'};
mcol = [4.6 7 8.6 10.4 11.6 5.6];  mrow = [4.4 1 4.4 1 4.4 5.4];
for i = 1:6
    add(INR, mass_nm{i}, mcol(i), mrow(i), 'InertiaType', 'PointMass', 'Mass', sprintf('m_pts(%d)', i));
    con(mdl, mass_at{i}, 'RConn', 1, mass_nm{i}, 'RConn', 1);
end

% --- where the pen tip really is (the "first check" of the 10/19 exercise)
add('sm_lib/Frames and Transforms/Transform Sensor', 'TCP sensor', 12.2, 3, 'MeasurementFrame', 'World', 'SenseX', 'on', 'SenseY', 'on', 'SenseZ', 'on');
con(mdl, 'World', 'RConn', 1, 'TCP sensor', 'LConn', 1);
con(mdl, 'tool', 'RConn', 1, 'TCP sensor', 'RConn', 1);
xyz = {'x', 'y', 'z'};
for k = 1:3
    add(P2S, ['out_' xyz{k}], 13.3, 2 + k, 'Unit', 'm');
    add('simulink/Sinks/To Workspace', ['tcp_' xyz{k}], 14.2, 2 + k, 'VariableName', ['tcp_' xyz{k}], 'SaveFormat', 'Timeseries');
    con(mdl, 'TCP sensor', 'RConn', 1 + k, ['out_' xyz{k}], 'LConn', 1);
    line_sig(mdl, ['out_' xyz{k}], 1, ['tcp_' xyz{k}], 1);
end

% --- solver: all motions are given, so nothing is integrated; results are saved at t_out
set_param(mdl, 'Solver', 'ode23t', 'MaxStep', '0.01', 'OutputOption', 'SpecifiedOutputTimes', 'OutputTimes', 't_out', ...
          'StopTime', 't_out(end)', 'SimMechanicsOpenEditorOnUpdate', 'off', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl, fullfile(folder, [mdl '.slx']));
close_system(mdl, 0);
end

function con(mdl, a, sa, ia, b, sb, ib)
% Physical connection between two ports (frames or physical signals).
pa = get_param([mdl '/' a], 'PortHandles');  pb = get_param([mdl '/' b], 'PortHandles');
add_line(mdl, pa.(sa)(ia), pb.(sb)(ib), 'autorouting', 'smart');
end

function line_sig(mdl, a, ia, b, ib)
% Ordinary Simulink signal line.
pa = get_param([mdl '/' a], 'PortHandles');  pb = get_param([mdl '/' b], 'PortHandles');
add_line(mdl, pa.Outport(ia), pb.Inport(ib), 'autorouting', 'smart');
end
