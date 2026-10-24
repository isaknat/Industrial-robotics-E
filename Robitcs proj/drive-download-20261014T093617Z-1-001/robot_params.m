function L = robot_params()
% Dimensions of the R-P-R-R robot (Project PDF p.2). Every script reads them
% from here, so a change in one place updates all steps.
%
% Every value is a PLACEHOLDER until the group decides (see the open-decision
% list in 学习计划_20261009.md). Current numbers: a working example from step 2.

L.h     = 200;          % [mm]  shoulder height: top of the column, where the rail starts (on the axis)
L.alpha = deg2rad(10);  % [rad] rail angle above horizontal (PDF p.2: alpha); negative = rail tilted down
L.L3    = 250;          % [mm]  q3 joint -> q4 joint
L.L4    = 100;          % [mm]  q4 joint -> pen tip (TCP)
L.sol   = -1;           % inverse kinematics solution: -1 = q3 joint nearer the axis, +1 = farther out

% Joint limits (mechanical stops). Not designed yet (step 6): chosen to cover
% the E from step 2 with some margin.                            PLACEHOLDER
L.qmin  = [-pi;  30; deg2rad(-100); deg2rad(-120)];   % [rad; mm; rad; rad]
L.qmax  = [ pi; 180; deg2rad( 100); deg2rad( 120)];

% Characteristic length used to make the score (1/condition number) compare
% joints of different nature (slider in mm, others in rad) and the pen angle
% with the pen position (Lez_04 p.24: normalize before comparing).   PLACEHOLDER
L.Lc    = 250;          % [mm]
end
