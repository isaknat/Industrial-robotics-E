function build_rprr_simscape_cad(mdl, folder, geom_dir)
% Builds the second Simscape Multibody model of the R-P-R-R robot: the CAD bodies and the drives.
% after CAD edition 1 (2026-11-24). Saved as <folder>/<mdl>.slx; step7b_simscape_cad rebuilds it on every run.
%
% Same joints, frames, zero positions and signs as build_rprr_simscape (the first version, point
% masses), so the joint motions of step 4 go in unchanged. What is different:
%  - Bodies: each moving sub-assembly of the Fusion model is one rigid body (File Solid). Its shape
%    is the STEP file exported from Fusion (for display only); its mass, centre of mass and full
%    inertia tensor are given as numbers, taken from Fusion (Fusion/step7_export_bodies.py), because
%    one density for the whole sub-assembly would be wrong (aluminium, steel, motors).
%      D_02 column and rail -> turns with q1      D_03 slider (shoulder carriage) -> moves with q2
%      D_04 q3 link with the long wrist belt  D_05 pen head (the tool) -> turns with q4
%    D_01 (base) and the bucket are fixed and shown only.
%  - Payload: a point mass at the pen tip (TCP), as before.
%  - Drives (assistant, 10/26: what changes the dynamics must be in the model, motors and transmissions
%    as inertia, not drawn): each joint has an "equivalent rotor", a spinning inertia J_r(j) mounted on
%    the body that carries the motor (q1: base, q2: column and rail, q3: slider, q4: q3 link), at the
%    motor, turned about the motor shaft at the motor's speed (wrist: 6*q4 + 3*q3). J_r(j) is the rotor plus every
%    turning part of the transmission (couplings, shafts, pulleys, belts, screw), each reduced to the
%    motor speed; step7b_simscape_cad computes it. Its torque, sensed, is the part of the motor torque
%    that accelerates the drive.
%    The rotor joints are given their motion like the robot joints, so no gear constraint is needed:
%    motor torque = tau*(joint load) + (rotor torque), tau = joint speed / motor speed. For timing belts
%    this is exact (input and output turn the same way); the masses of the drive parts are already in
%    the bodies.
%
% Variables set by step7b_simscape_cad (SI units):
%   h_m, alpha_deg, L3_m, L4_m, g, t_out          as in the first version
%   q_1..q_4, w_1..w_4, b_1..b_4                  [t, value] position, speed, acceleration of each joint
%   m_pay                                         payload at the pen tip [kg]
%   m_X, c_X, Im_X, Ip_X  (X = D01 ... D05, drum) mass, centre of mass in the body's CAD frame,
%                                                 [Ixx Iyy Izz] and [Iyz Izx Ixy] about the centre of mass
%   R_W, R_23, R_34                               rotation from a joint frame to a CAD frame (below)
%   n_r (1x4), J_r (1x4)                          motor turns per joint unit [rad/rad, rad/m]; equivalent rotor [kg m^2]
%   p_r1..p_r4, R_r1..R_r4                        where each motor sits in its body's CAD frame, and the
%                                                 turn that lays the rotor joint's z on the motor shaft
%
% CAD frames (Fusion, Y up) against the joint frames of the first version (z up, arm along +y at q1 = 0):
%   world / after q1 -> D_01, bucket, D_02:  CAD X = arm direction (+y), CAD Y = up (+z), CAD Z = +x   (R_W)
%   after q2 -> D_03: origin on the q3 axis, same directions as D_02                               (R_23)
%   after q3 -> D_04, after q4 -> D_05: origin on the joint axis, turned 180 deg about the joint axis,
%   because the joint frames have the link along -x and the CAD frames along +X                     (R_34)
% Step7b checks these against Fusion in three poses (gravity torques at q3 and q4, inertia about q1).

if bdIsLoaded(mdl), close_system(mdl, 0); end
old = fullfile(folder, [mdl '.slx']);
if exist(old, 'file'), delete(old); end
load_system('sm_lib');  load_system('nesl_utility');
new_system(mdl);
S2P = ['nesl_utility/Simulink-PS' newline 'Converter'];
P2S = ['nesl_utility/PS-Simulink' newline 'Converter'];
RT  = 'sm_lib/Frames and Transforms/Rigid Transform';
INR = 'sm_lib/Body Elements/Inertia';
FS  = 'sm_lib/Body Elements/File Solid';
pos = @(col, row) [60 + 150*col, 60 + 95*row, 60 + 150*col + 70, 60 + 95*row + 50];
add = @(lib, name, col, row, varargin) add_block(lib, [mdl '/' name], 'Position', pos(col, row), varargin{:});

% --- world, gravity, solver
add('sm_lib/Frames and Transforms/World Frame', 'World', 0, 3);
add('sm_lib/Utilities/Mechanism Configuration', 'Gravity', 0, 4, 'GravityVector', '[0 0 -g]');
add(['nesl_utility/Solver' newline 'Configuration'], 'Solver', 0, 5);
con(mdl, 'World', 'RConn', 1, 'Gravity', 'RConn', 1);
con(mdl, 'World', 'RConn', 1, 'Solver', 'RConn', 1);

% --- robot joints, as in the first version
jtype = {'Revolute', 'Prismatic', 'Revolute', 'Revolute'};  junit = {'rad', 'm', 'rad', 'rad'};
jcol  = [1 4 7 10];
names = {'q', 'w', 'b'};
for j = 1:4
    jn = sprintf('q%d', j);
    add(['sm_lib/Joints/' jtype{j} ' Joint'], jn, jcol(j), 3, ...
        'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque', 'SenseTorqueForce', 'on');
    for k = 1:3
        nm = sprintf('%s_%d', names{k}, j);
        add_block('simulink/Sources/From Workspace', [mdl '/' nm], 'VariableName', nm, 'SampleTime', '0', ...
            'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value', ...
            'Position', [60 + 150*(jcol(j) - 1) + 55*(k - 1), 30 + 45*(k - 1), 60 + 150*(jcol(j) - 1) + 55*(k - 1) + 45, 30 + 45*(k - 1) + 28]);
    end
    add(S2P, sprintf('in_q%d', j), jcol(j), 1.6, 'FilteringAndDerivatives', 'provide', 'UdotUserProvided', '2', 'Unit', junit{j});
    for k = 1:3, line_sig(mdl, sprintf('%s_%d', names{k}, j), 1, sprintf('in_q%d', j), k); end
    con(mdl, sprintf('in_q%d', j), 'RConn', 1, jn, 'LConn', 2);
    add(P2S, sprintf('out_q%d', j), jcol(j) + 0.2, 5);
    add('simulink/Sinks/To Workspace', sprintf('F_q%d', j), jcol(j) + 0.2, 6, 'VariableName', sprintf('F_q%d', j), 'SaveFormat', 'Timeseries');
    con(mdl, jn, 'RConn', 2, sprintf('out_q%d', j), 'LConn', 1);
    line_sig(mdl, sprintf('out_q%d', j), 1, sprintf('F_q%d', j), 1);
end
add(RT, 'to shoulder', 2, 3, 'TranslationMethod', 'StandardAxis', 'TranslationStandardAxis', '+Z', 'TranslationStandardOffset', 'h_m');
add(RT, 'rail direction', 3, 3, 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+X', 'RotationAngle', 'alpha_deg - 90');
add(RT, 'q3 axis', 6, 3, 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+Y', 'RotationAngle', '90');
add(RT, 'q3 link', 9, 3, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[-L3_m 0 0]');
add(RT, 'tool', 12, 3, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[-L4_m 0 0]');
con(mdl, 'World', 'RConn', 1, 'q1', 'LConn', 1);
con(mdl, 'q1', 'RConn', 1, 'to shoulder', 'LConn', 1);
con(mdl, 'to shoulder', 'RConn', 1, 'rail direction', 'LConn', 1);
con(mdl, 'rail direction', 'RConn', 1, 'q2', 'LConn', 1);
con(mdl, 'q2', 'RConn', 1, 'q3 axis', 'LConn', 1);
con(mdl, 'q3 axis', 'RConn', 1, 'q3', 'LConn', 1);
con(mdl, 'q3', 'RConn', 1, 'q3 link', 'LConn', 1);
con(mdl, 'q3 link', 'RConn', 1, 'q4', 'LConn', 1);
con(mdl, 'q4', 'RConn', 1, 'tool', 'LConn', 1);

% --- CAD bodies: a frame turned onto the CAD frame, then the File Solid with the CAD mass properties
body = {   % block name,           variable suffix, attached to (block, port), turn, STEP file,       colour,           opacity
    'D_01 base',               'D01',  'World', 'RConn', 'R_W',  'D_01.step',      [0.55 0.55 0.6],  1
    'bucket',                  'drum', 'World', 'RConn', 'R_W',  'Wave_Drum.step', [0.6 0.75 0.9],   0.25
    'D_02 column and rail',    'D02',  'q1',    'RConn', 'R_W',  'D_02.step',      [0.75 0.75 0.8],  1
    'D_03 slider',             'D03',  'q2',    'RConn', 'R_23', 'D_03.step',      [0.85 0.6 0.35],  1
    'D_04 q3 link',            'D04',  'q3',    'RConn', 'R_34', 'D_04.step',      [0.35 0.55 0.85], 1
    'D_05 pen head (tool)',    'D05',  'q4',    'RConn', 'R_34', 'D_05.step',      [0.85 0.3 0.3],   1};
for b = 1:size(body, 1)
    fr = ['to ' body{b, 1} ' frame'];
    add(RT, fr, 1 + 2*(b - 1), 7.5, 'RotationMethod', 'RotationMatrix', 'RotationMatrix', body{b, 5});
    con(mdl, body{b, 3}, body{b, 4}, 1, fr, 'LConn', 1);
    sfx = body{b, 2};
    add(FS, body{b, 1}, 1 + 2*(b - 1), 8.5, 'ExtGeomFileName', fullfile(geom_dir, body{b, 6}), ...
        'InertiaType', 'Custom', 'Mass', ['m_' sfx], 'CenterOfMass', ['c_' sfx], ...
        'MomentsOfInertia', ['Im_' sfx], 'ProductsOfInertia', ['Ip_' sfx], ...
        'GraphicDiffuseColor', mat2str(body{b, 7}), 'GraphicOpacity', num2str(body{b, 8}));
    con(mdl, fr, 'RConn', 1, body{b, 1}, frame_port(mdl, body{b, 1}), 1);
end

% --- payload at the pen tip
add(INR, 'payload at the tip', 13, 4.4, 'InertiaType', 'PointMass', 'Mass', 'm_pay');
con(mdl, 'tool', 'RConn', 1, 'payload at the tip', 'RConn', 1);

% --- equivalent rotors of the four drives
carrier = {'to D_01 base frame', 'to D_02 column and rail frame', 'to D_03 slider frame', 'to D_03 slider frame'};
for j = 1:4
    mo = sprintf('motor %d', j);  ro = sprintf('rotor %d', j);
    add(RT, mo, 1 + 3*(j - 1), 10, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', sprintf('p_r%d', j), ...
        'RotationMethod', 'RotationMatrix', 'RotationMatrix', sprintf('R_r%d', j));
    con(mdl, carrier{j}, 'RConn', 1, mo, 'LConn', 1);
    add('sm_lib/Joints/Revolute Joint', ro, 2 + 3*(j - 1), 10, ...
        'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque', 'SenseTorqueForce', 'on');
    con(mdl, mo, 'RConn', 1, ro, 'LConn', 1);
    add(INR, sprintf('rotor %d inertia', j), 3 + 3*(j - 1), 10, 'InertiaType', 'Custom', 'Mass', '1e-9', ...
        'MomentsOfInertia', sprintf('[J_r(%d)/2 J_r(%d)/2 J_r(%d)]', j, j, j));
    con(mdl, ro, 'RConn', 1, sprintf('rotor %d inertia', j), 'RConn', 1);
    % the rotor turns n_r(j) times the joint: the same three signals, scaled
    for k = 1:3
        gk = sprintf('n%d_%s', j, names{k});
        add_block('simulink/Math Operations/Gain', [mdl '/' gk], 'Gain', sprintf('n_r(%d)', j), ...
            'Position', [60 + 150*(1 + 3*(j - 1)) + 40*(k - 1), 60 + 95*11.3, 60 + 150*(1 + 3*(j - 1)) + 40*(k - 1) + 30, 60 + 95*11.3 + 25]);
        source = sprintf('%s_%d', names{k}, j);
        if j == 4
            half = ['shoulder_half_' names{k}]; combined = ['wrist_combined_' names{k}];
            add_block('simulink/Math Operations/Gain', [mdl '/' half], 'Gain', '0.5', ...
                'Position', [1400, 1210+50*k, 1430, 1230+50*k]);
            add_block('simulink/Math Operations/Sum', [mdl '/' combined], 'Inputs', '++', ...
                'Position', [1480, 1210+50*k, 1510, 1230+50*k]);
            line_sig(mdl, sprintf('%s_3', names{k}), 1, half, 1);
            line_sig(mdl, half, 1, combined, 1);
            line_sig(mdl, source, 1, combined, 2);
            source = combined;
        end
        line_sig(mdl, source, 1, gk, 1);
    end
    add(S2P, sprintf('in_r%d', j), 2 + 3*(j - 1), 11.3, 'FilteringAndDerivatives', 'provide', 'UdotUserProvided', '2', 'Unit', 'rad');
    for k = 1:3, line_sig(mdl, sprintf('n%d_%s', j, names{k}), 1, sprintf('in_r%d', j), k); end
    con(mdl, sprintf('in_r%d', j), 'RConn', 1, ro, 'LConn', 2);
    add(P2S, sprintf('out_r%d', j), 2.4 + 3*(j - 1), 12.4);
    add('simulink/Sinks/To Workspace', sprintf('T_r%d', j), 3.2 + 3*(j - 1), 12.4, 'VariableName', sprintf('T_r%d', j), 'SaveFormat', 'Timeseries');
    con(mdl, ro, 'RConn', 2, sprintf('out_r%d', j), 'LConn', 1);
    line_sig(mdl, sprintf('out_r%d', j), 1, sprintf('T_r%d', j), 1);
end

% --- where the pen tip really is
add('sm_lib/Frames and Transforms/Transform Sensor', 'TCP sensor', 13, 3, 'MeasurementFrame', 'World', 'SenseX', 'on', 'SenseY', 'on', 'SenseZ', 'on');
con(mdl, 'World', 'RConn', 1, 'TCP sensor', 'LConn', 1);
con(mdl, 'tool', 'RConn', 1, 'TCP sensor', 'RConn', 1);
xyz = {'x', 'y', 'z'};
for k = 1:3
    add(P2S, ['out_' xyz{k}], 14, 2 + k, 'Unit', 'm');
    add('simulink/Sinks/To Workspace', ['tcp_' xyz{k}], 15, 2 + k, 'VariableName', ['tcp_' xyz{k}], 'SaveFormat', 'Timeseries');
    con(mdl, 'TCP sensor', 'RConn', 1 + k, ['out_' xyz{k}], 'LConn', 1);
    line_sig(mdl, ['out_' xyz{k}], 1, ['tcp_' xyz{k}], 1);
end

set_param(mdl, 'Solver', 'ode23t', 'MaxStep', '0.01', 'OutputOption', 'SpecifiedOutputTimes', 'OutputTimes', 't_out', ...
          'StopTime', 't_out(end)', 'SimMechanicsOpenEditorOnUpdate', 'off', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl, fullfile(folder, [mdl '.slx']));
close_system(mdl, 0);
end

function p = frame_port(mdl, blk)
% The File Solid's reference-frame port (the library places it on the right or the left).
ph = get_param([mdl '/' blk], 'PortHandles');
if ~isempty(ph.RConn), p = 'RConn'; else, p = 'LConn'; end
end

function con(mdl, a, sa, ia, b, sb, ib)
pa = get_param([mdl '/' a], 'PortHandles');  pb = get_param([mdl '/' b], 'PortHandles');
add_line(mdl, pa.(sa)(ia), pb.(sb)(ib), 'autorouting', 'smart');
end

function line_sig(mdl, a, ia, b, ib)
pa = get_param([mdl '/' a], 'PortHandles');  pb = get_param([mdl '/' b], 'PortHandles');
add_line(mdl, pa.Outport(ia), pb.Inport(ib), 'autorouting', 'smart');
end
